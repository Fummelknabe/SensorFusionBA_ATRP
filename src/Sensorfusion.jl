using DSP
using LinearAlgebra


predictedStates = StructArray(PositionalState[])

rateCameraConfidence(cc, exponent) = cc^exponent

# linearized system matrices
F_c(state::PositionalState, u::PositionalData) = [1 0 0 -u.deltaTime*state.v*sin(state.Ψ) 0;
                                                  0 1 0 u.deltaTime*state.v*cos(state.Ψ) 0;
                                                  0 0 1 0 u.deltaTime*state.v*cos(state.θ);
                                                  0 0 0 1 0;
                                                  0 0 0 0 1]

F_g(δt::Float32) = [1 δt;
                    0 1]

const H_g = [0 1]

const H_c = [1 0 0 0 0;
             0 1 0 0 0;
             0 0 1 0 0]

P(F, Q, oldP, size) = F*oldP*transpose(F) .+ Q*Matrix(I, size, size)
K(P, H, R, size) = P*transpose(H)*(H*P*transpose(H) .+ R*Matrix(I, size, size))^-1

"""
This function transforms camera coords ontop of the prediction. This should be 
unnecessary if the correct initial transform is choosen for the camera. 
(for old data, this has to be used)
DISCLAIMER: this function does not transform the orientation of the camera.
"""
function transformCameraCoords(cameraCoords::Vector{Float32}, Ψ::Float32)
      # Create 2D rotational matrix
      R = [cos(Ψ - π/2) -sin(Ψ - π/2);
           sin(Ψ - π/2) cos(Ψ - π/2)]

      temp = R*cameraCoords[1:2]

      return [temp[1], temp[2], cameraCoords[3], cameraCoords[4]]
end

"""
This function transforms a given position so that the z-vector points up in the intertial frame 

# Arguments
`pos::Vector{Float32}`: The position to transform
`up::Vector{Float32}`: The vector that holds the acceleration data 

# Returns 
`Vector{Float32}`: The transformed position
"""
function transformCoords(pos::Vector{T}, up::Vector{T}, angularSpeed::Vector{T}, weights::Tuple{T, T}, δt::T) where {T <: Real}
    # Projection in planes
    projXZ = [up[1], 0.0, -up[3]]
    projYZ = [0.0, up[2], -up[3]]

    # Angle between axis
    α = weights[1] * -acos(projXZ[3] / norm(projXZ)) + weights[2] * angularSpeed[2]*δt 
    β = weights[1] * -acos(projYZ[3] / norm(projYZ)) + weights[2] * angularSpeed[1]*δt

    Rx = [1     0      0;
          0 cos(β) -sin(β); 
          0 sin(β) cos(β)]    
    Ry = [cos(α) 0 sin(α);
          0      1      0;
          -sin(α) 0 cos(α)]

    return (Rx*Ry)*pos
end

function computeSpeed(cameraChange::Vector{Float32}, δt::Vector{Float32}, v::Float32, ratedCamConfidence::Float32)
      length(cameraChange) == length(δt) || throw("Computing Speed failed, as $(cameraChange) and $(δt) were not the same length.")
      
      # Filter speed from camera change
      l = length(cameraChange)
      kernel = gaussian(l, 1/3)
      kernel = kernel ./ sum(kernel)
      cameraSpeed = conv(cameraChange ./ δt, kernel)
      
      # Combine with wheel odometry speed value
      return Float32((1-ratedCamConfidence) * v + ratedCamConfidence * cameraSpeed[l])
end

# Calculate angles using angular Velocity ω
Ψ(oldΨ, δt, ω::Vector{Float32}) = oldΨ - δt*ω[3]
θ_ang(oldθ, δt, ω::Vector{Float32}) = oldθ - δt*ω[2]

# Calculate angles using steering angle δ and acceleration
Ψ(oldΨ, δt, δ::Float32) = oldΨ + δt*δ
θ_acc(oldθ, δt, a::Vector{Float32}) = oldθ + δt*acos(a[3]/norm(a))

function changeInPosition(a::Vector{Float32}, v::Float32, Ψ::Float32, θ::Float32, δt::Float32)
      # Get Change in Position
      x_dot = v * cos(Ψ)
      y_dot = v * sin(Ψ)

      # Only when a significant deviation from z-Axis = 1g
      z_dot = (abs(a[3] - 1) > 0.02) ? v * sin(θ) : 0

      return [x_dot*δt, y_dot*δt, z_dot*δt]
end

"""
This function predicts the next position from given datapoints and the last positinal state.

# Arguments
- `posState::PositionalState`: The last positional state. 
- `dataPoints::StructVector{PositionalData}`: An array of datapoints. There should atleast 2 data points to more accurately predict position. 

# Returns
- `PositionalState`: The new positional state of the robot.
"""
function predict(posState::PositionalState, dataPoints::StructVector{PositionalData}, settings::PredictionSettings)
      # Get data from data point
      amountDataPoints = length(dataPoints)
      amountDataPoints > 1 || throw("More than $(amountDataPoints) Data Points need to be given to predict position.")
      newData = dataPoints[amountDataPoints]

      camPosMatrix = reduce(vcat, transpose.(dataPoints.cameraPos))   
      v = computeSpeed(camPosMatrix[:, 4], dataPoints.deltaTime, newData.sensorSpeed, rateCameraConfidence(newData.cameraConfidence, settings.exponentCC))
      
      # Apply Kalman Filter to angular velocity data if wanted
      P_g_update = Matrix(I, 2, 2)
      if settings.kalmanFilterGyro
            P_predict = P(F_g(newData.deltaTime), settings.processNoiseG, posState.P_g, 2)
            kalmanGain = K(P_predict, H_g, settings.measurementNoiseG, 1)
            P_g_update = P_predict .- kalmanGain*H_g*P_predict
            Ψ_ang = posState.Ψ + kalmanGain[2,1] * (settings.steerAngleFactor*(newData.steerAngle-120) - newData.imuGyro[3])
      end

      # Calculate delta position with different information
      δOdoSteeringAngle = changeInPosition(newData.imuAcc, 
                                           v, 
                                           Ψ(posState.Ψ, newData.deltaTime, Float32(settings.steerAngleFactor*(newData.steerAngle-120))),
                                           θ_acc(posState.θ, newData.deltaTime, newData.imuAcc),
                                           newData.deltaTime)

      δOdoAngularVelocity = changeInPosition(newData.imuAcc, 
                                             v, 
                                             settings.kalmanFilterGyro ? Float32(Ψ_ang) : Ψ(posState.Ψ, newData.deltaTime, newData.imuGyro),
                                             θ_ang(posState.θ, newData.deltaTime, newData.imuGyro),
                                             newData.deltaTime)

      δOdoCompassCourse = changeInPosition(newData.imuAcc,
                                           v,
                                           -convertMagToCompass(newData.imuMag),
                                           θ_ang(posState.θ, newData.deltaTime, newData.imuGyro),
                                           newData.deltaTime)

      δCamPos = dataPoints[amountDataPoints].cameraPos[1:3] - dataPoints[amountDataPoints - 1].cameraPos[1:3]

      ratedCC = rateCameraConfidence(newData.cameraConfidence, settings.exponentCC)
      δodometryPos = (settings.odoGyroFactor*δOdoAngularVelocity + settings.odoSteerFactor*δOdoSteeringAngle + settings.odoMagFactor*δOdoCompassCourse) / (settings.odoGyroFactor + settings.odoMagFactor + settings.odoSteerFactor)

      newPosition = posState.position + (1-ratedCC)*δodometryPos + ratedCC*δCamPos
      P_c_update = Matrix(I, 5, 5)
      if settings.kalmanFilterCamera
            P_predict = P(F_c(posState, dataPoints[amountDataPoints]), settings.processNoiseC, posState.P, 5)
            kalmanGain = K(P_predict, H_c, settings.measurementNoiseC, 3)
            P_c_update = P_predict .- kalmanGain*H_c*P_predict
            newPosition = posState.position + δodometryPos + kalmanGain[1:3, 1:3] * (dataPoints[amountDataPoints].cameraPos[1:3] - (posState.position + δodometryPos))
      end

      return PositionalState(newPosition,
                             v,
                             P_c_update,
                             P_g_update,
                             (settings.odoSteerFactor*Ψ(posState.Ψ, newData.deltaTime, Float32(settings.steerAngleFactor*(newData.steerAngle-120)))+settings.odoGyroFactor*(settings.kalmanFilterGyro ? Float32(Ψ_ang) : Ψ(posState.Ψ, newData.deltaTime, newData.imuGyro))+settings.odoMagFactor*(-convertMagToCompass(newData.imuMag))) / (settings.odoGyroFactor + settings.odoMagFactor + settings.odoSteerFactor), 
                             (settings.odoSteerFactor*θ_acc(posState.θ, newData.deltaTime, newData.imuAcc)+settings.odoGyroFactor*θ_ang(posState.θ, newData.deltaTime, newData.imuGyro)+settings.odoMagFactor*θ_ang(posState.θ, newData.deltaTime, newData.imuGyro)) / (settings.odoGyroFactor + settings.odoMagFactor + settings.odoSteerFactor))
end

"""
Coverts data from magnetometer to campass course.

# Arguments 
- `magnetometerVector::Vector{Float32}`: The magnetometer data from the IMU 
- `accelerometerVector::Vector{Float32}`: The accelerometer data from the IMU
# Returns
- `Float32`: The compass course in radians
"""
function convertMagToCompass(magnetometerVector::Vector{Float32}; accelerometerVector::Union{Vector{Float32}, Nothing}=nothing)
      northVector = magnetometerVector
      if !isnothing(accelerometerVector)
            downVector = [accelerometerVector[1], -accelerometerVector[2], accelerometerVector[3]]
            northVector = magnetometerVector - (downVector * (dot(magnetometerVector, downVector) / dot(downVector, downVector)))
      end
      angle = atan(northVector[1], northVector[2])

      return Float32((angle > 0) ? angle : angle + 2*π)
end

"""
This function predicts position from recorded data.

# Arguments
- `posData::StructVector{PositionalData}`: The recorded data points.
"""
function predictFromRecordedData(posData::StructVector{PositionalData}, settings::PredictionSettings)
      # Set up predicted states and initialize first one
      predictedStates = StructArray(PositionalState[])
      push!(predictedStates, PositionalState(posData[1].cameraPos[1:3], posData[1].sensorSpeed, Matrix(I, 5, 5), Matrix(I, 2, 2), convertMagToCompass(posData[1].imuMag), θ_acc(0.0, 1.0, posData[1].imuAcc)))

      # Predict for every coming positional value
      for i in 2:length(posData)
            push!(predictedStates, predict(
                  predictedStates[i-1],
                  # give mutiple positional data points if possible
                  (i > 9) ? posData[(i-9):i] : posData[(i-1):i],
                  settings
            ))
      end

      return predictedStates
end

"""
Start sensor fusion with a continuous flow of data.

# Arguments
- `posData::StructVector{PositionalData}`: The last recorded data points. Length should be more than 1.
"""
function initializeSensorFusion(posData::StructVector{PositionalData}, settings::PredictionSettings)
      if length(predictedStates) == 0
            # Set first state
            global predictedStates[1] = PositionalState(posData[1].cameraPos[1:3], posData[1].sensorSpeed, Matrix(I, 5, 5), Matrix(I, 2, 2), convertMagToCompass(posData[1].imuMag), θ_acc(0.0, 1.0, posData[1].imuAcc))
      end

      push!(predictedStates, predict(predictedStates[length(predictedStates)  - 1], posData, settings))
end