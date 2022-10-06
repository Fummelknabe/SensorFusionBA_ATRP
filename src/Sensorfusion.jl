using DSP
using LinearAlgebra

predictedStates = StructArray(PositionalState[])

rateCameraConfidence(cc, exponent, useSin::Bool) = Float32(useSin ? sin(π/2 * cc)^exponent : cc^exponent)

# linearized system matrices
F_c(state::PositionalState, u::PositionalData, β::Float32) = [1 0 0 -u.deltaTime*state.v*cos(state.θ)*sin(state.Ψ+β) -u.deltaTime*state.v*sin(state.θ)*cos(state.Ψ+β);
                                                              0 1 0 u.deltaTime*state.v*cos(state.θ)*cos(state.Ψ+β) -u.deltaTime*state.v*sin(state.θ)*sin(state.Ψ+β);
                                                              0 0 1 0 u.deltaTime*state.v*cos(state.θ);
                                                              0 0 0 1 0;
                                                              0 0 0 0 1]

F_g(δt::Float32) = [1 δt;
                    0 1]

const H_g = [0 1]

const H_c = [1 0 0 0 0;
             0 1 0 0 0;
             0 0 1 0 0]

const Hₛ = [1 0 0 0 0;
           0 1 0 0 0;
           0 0 1 0 0;
           0 0 0 1 0;
           0 0 0 0 1]

const lᵥ = 0.18
const lₕ = 0.50

# Dimension of state for UKF
const n = 5

# Calculate angles using angular Velocity ω
Ψ(oldΨ, δt, ω::Vector{Float32}) = oldΨ - δt*ω[3]
θ_ang(oldθ, δt, ω::Vector{Float32}) = oldθ + δt*ω[2]
ϕ_ang(oldϕ, δt, ω::Vector{Float32}) = oldϕ + δt*ω[1]

# Calculate angles using steering angle δ and acceleration
Ψ(oldΨ, δt, δ::Float32, β::Float32, v::Float32) = Float32(oldΨ + δt*(v/(lₕ+lᵥ)*cos(β)*tan(δ)))
function θ_ϕ_acc(a::Vector{Float32}, Ψ::Float32)
      # rotate horizontal axis by yaw angle
      x_plane = [cos(Ψ), sin(Ψ), 0]

      y_v = cross(a, x_plane)
      x_v = cross(y_v, a)

      θ = sign(x_v[3])*Float32(acos(round(dot(x_v, x_plane)/(norm(x_v)*norm(x_plane)); digits=3)))
      ϕ = sign(y_v[3])*Float32(acos(round(dot(y_v, [y_v[1], y_v[2], 0])/(norm(y_v)*norm([y_v[1], y_v[2], 0])); digits=3)))

      return θ, ϕ
end

β(δ) = Float32(atan(lₕ/(lᵥ+lₕ)*tan(δ)))

P(F, Q, oldP, size) = F*oldP*transpose(F) .+ Q*Matrix(I, size, size)
K(P, H, R, size) = P*transpose(H)*(H*P*transpose(H) .+ R*Matrix(I, size, size))^-1

function changeInPosition(a::Vector{Float32}, v::Float32, Ψ::Float32, θ::Float32, δt::Float32; β::Float32=Float32(0.0))
      # Only when a significant deviation from z-Axis = 1g 
      θᵢₙ = abs(a[3] - 1) > 0.02
      
      # Get Change in Position
      ẋ = (θᵢₙ ? cos(θ) : 1) * v * cos(Ψ + β)
      ẏ = (θᵢₙ ? cos(θ) : 1) * v * sin(Ψ + β)

      ż = (θᵢₙ) ? v * -sin(θ) : 0

      return [ẋ*δt, ẏ*δt, ż*δt]
end

include("UKF.jl")

"""
This function transforms camera coords ontop of the prediction. This should be 
unnecessary if the correct initial transform is choosen for the camera. 
(for old data, this has to be used)
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

"""
This function computes the speed from measured speed and camera position.
"""
function computeSpeed(cameraMatrix::Matrix{Float32}, δt::Vector{Float32}, v::Float32, ratedCamConfidence::Float32, σ::Float32, command::String, pastBackwards::Bool, δ::Float32)
      cameraChange = cameraMatrix[:, 4]
      length(cameraChange) == length(δt) || throw("Computing Speed failed, as $(cameraChange) and $(δt) were not the same length.")
      sign = pastBackwards ? -1 : 1

      # Wheel slippage
      ws = false

      if length(cameraChange) > 2 && ratedCamConfidence > 0.6
            # get direction of robot from camera
            posChange = cameraMatrix[end, 1:2] - cameraMatrix[end-1, 1:2]
            prevPosChange = cameraMatrix[end-1, 1:2] - cameraMatrix[end-2, 1:2]
            α = acos(round(dot(posChange, prevPosChange) / (norm(posChange) * norm(prevPosChange)); digits=4))
            if α > 3/4*π  sign *= -1 end

            # check wheel slippage
            ws = α < π/2 && 2*α > δ
      else
            # direction change should only occur when speed is relatively small
            if occursin("backward", command) && abs(v) < 0.3
                  sign = -1
            elseif occursin("forward", command) && abs(v) < 0.3
                  sign = 1
            end            
      end
      
      # Filter speed from camera change
      l = length(cameraChange)
      kernel = gaussian(l, σ)
      kernel = kernel ./ sum(kernel)
      cameraSpeed = conv(cameraChange ./ δt, kernel)
      
      # Combine with wheel odometry speed value
      return sign*(ws ? Float32(cameraSpeed[l]) : (Float32((1-ratedCamConfidence) * v + ratedCamConfidence * cameraSpeed[l]))), ws
end

function extractTaitbryanFromOrientation(state::PositionalState, dataPoints::StructVector{PositionalData})
      cameraOrientation = (dataPoints[end].cameraOri, dataPoints[end-1].cameraOri)

      δt = dataPoints[end].deltaTime
      Ψᵥₒ = state.Ψ + δt*(cameraOrientation[1][2] - cameraOrientation[2][2])
      θᵥₒ = state.θ + δt*(cameraOrientation[1][1] - cameraOrientation[2][1])
      ϕᵥₒ = state.ϕ + δt*(cameraOrientation[1][3] - cameraOrientation[2][3])

      return Ψᵥₒ, θᵥₒ, ϕᵥₒ
end

"""
This function predicts the next position from given datapoints and the last positinal state.

# Arguments
- `posState::PositionalState`: The last positional state. 
- `dataPoints::StructVector{PositionalData}`: An array of datapoints. There should atleast 2 data points to more accurately predict position. 

# Returns
- `PositionalState`: The new positional state of the robot.
"""
function predict(posState::PositionalState, dataPoints::StructVector{PositionalData}, settings::PredictionSettings; #=onyl for debugging:=#iteration::Int=0)
      # Get data from data point
      amountDataPoints = length(dataPoints)
      amountDataPoints > 1 || throw("More than $(amountDataPoints) Data Points need to be given to predict position.")
      newData = dataPoints[amountDataPoints]

      camPosMatrix = reduce(vcat, transpose.(dataPoints.cameraPos))   
      v, ws = computeSpeed(camPosMatrix, dataPoints.deltaTime, newData.sensorSpeed, rateCameraConfidence(newData.cameraConfidence, settings.speedExponentCC, settings.speedUseSinCC), settings.σ_forSpeedKernel, newData.command, posState.v < 0 || (!(posState.v === 0) && posState.v == 0), Float32(newData.steerAngle*π/180))

      # Apply Kalman Filter to angular velocity data if wanted
      P_g_update = Matrix(I, 2, 2)
      if settings.kalmanFilterGyro
            P_predict = P(F_g(newData.deltaTime), settings.processNoiseG, posState.P_g, 2)
            kalmanGain = K(P_predict, H_g, settings.measurementNoiseG, 1)
            P_g_update = P_predict .- kalmanGain*H_g*P_predict
            current_δ = settings.steerAngleFactor*(newData.steerAngle*π/180)            
            Ψ_ang = posState.Ψ + kalmanGain[2,1] * (newData.deltaTime*(v/(lₕ+lᵥ)*cos(β(current_δ))*tan(current_δ)) - newData.imuGyro[3])
      end

      ratedCC = rateCameraConfidence(newData.cameraConfidence, settings.exponentCC, settings.useSinCC)

      # Calculate orientation 
      Ψ_acc = Ψ(posState.Ψ, newData.deltaTime, Float32(settings.steerAngleFactor*(newData.steerAngle*π/180)), β(newData.steerAngle*π/180), v)
      θ_acc, ϕ_acc = θ_ϕ_acc(newData.imuAcc, Ψ_acc)

      # Calculate delta position with different information
      if !settings.UKF
            δOdoSteeringAngle = changeInPosition(newData.imuAcc, 
                                                v, 
                                                Ψ_acc,
                                                θ_acc,
                                                newData.deltaTime,
                                                β=Float32(β(newData.steerAngle*π/180)))
      else
            #@info "Iteration: $(iteration)"
            try   # JUST DEBUGGING
                  wₘ = computeWeights(true, settings)
                  wₖ = computeWeights(false, settings)
                  μₜ̇, Χₜ, Σₜ̇ = UKF_prediction(Float32.([posState.position[1], posState.position[2], posState.position[3], posState.Ψ, posState.θ]),
                                          wₘ,
                                          wₖ,
                                          posState.Χ,
                                          Float32.([newData.imuGyro[1], newData.imuGyro[2], newData.imuGyro[3], v, newData.deltaTime, newData.steerAngle]),
                                          posState.Σ,
                                          settings)
                      
                  μₜ, Σₜ = UKF_update(μₜ̇, wₘ, wₖ, Χₜ, Σₜ̇, settings, [newData.cameraPos[1], newData.cameraPos[2], newData.cameraPos[3], convertMagToCompass(newData.imuMag), θ_acc], ratedCC)

                  δOdoSteeringAngle = μₜ[1:3] - posState.position

                  # Update state
                  posState.Σ = Σₜ
                  posState.Χ = Χₜ
            catch e     
                  @error "Error occured with: $(settings)."
                  println("Sigma Points: $(posState.Χ)")
                  println("covariance $(posState.Σ)")
                  println("lambda: $(settings.α^2*(n + settings.κ) - n)")
                  throw(e)
            end  
      end

      δOdoAngularVelocity = changeInPosition(newData.imuAcc, 
                                             v, 
                                             settings.kalmanFilterGyro ? Float32(Ψ_ang) : Ψ(posState.Ψ, newData.deltaTime, newData.imuGyro),
                                             θ_ang(posState.θ, newData.deltaTime, newData.imuGyro),
                                             newData.deltaTime)

      δOdoCompassCourse = changeInPosition(newData.imuAcc,
                                           v,
                                           convertMagToCompass(newData.imuMag, accelerometerVector=newData.imuAcc),
                                           θ_ang(posState.θ, newData.deltaTime, newData.imuGyro),
                                           newData.deltaTime)

      δCamPos = dataPoints[amountDataPoints].cameraPos[1:3] - dataPoints[amountDataPoints - 1].cameraPos[1:3]

      δodometryPos = (settings.odoGyroFactor.*δOdoAngularVelocity .+ settings.odoSteerFactor.*δOdoSteeringAngle .+ settings.odoMagFactor.*δOdoCompassCourse) ./ (settings.odoGyroFactor + settings.odoMagFactor + settings.odoSteerFactor)

      newPosition = posState.position + ((ws) ? δCamPos : (1-ratedCC)*δodometryPos + ratedCC*δCamPos)
      P_c_update = Matrix(I, 5, 5)
      if settings.kalmanFilterCamera
            P_predict = P(F_c(posState, dataPoints[amountDataPoints], Float32(β(newData.steerAngle*π/180))), settings.processNoiseC, posState.P_c, 5)
            kalmanGain = K(P_predict, H_c, settings.measurementNoiseC, 3)
            P_c_update = P_predict .- kalmanGain*H_c*P_predict
            newPosition = posState.position + δodometryPos + kalmanGain[1:3, 1:3] * (dataPoints[amountDataPoints].cameraPos[1:3] - (posState.position + δodometryPos))
      end

      Ψᵥₒ, θᵥₒ, ϕᵥₒ = extractTaitbryanFromOrientation(posState, dataPoints)
      Ψₒ = (settings.odoSteerFactor*Ψ_acc+settings.odoGyroFactor*(settings.kalmanFilterGyro ? Float32(Ψ_ang) : Ψ(posState.Ψ, newData.deltaTime, newData.imuGyro))+(settings.ΨₒmagInfluence ? settings.odoMagFactor*convertMagToCompass(newData.imuMag) : 0)) / (settings.odoGyroFactor + (settings.ΨₒmagInfluence ? settings.odoMagFactor : 0) + settings.odoSteerFactor)
      θ₀ = (settings.odoSteerFactor*θ_acc+settings.odoGyroFactor*θ_ang(posState.θ, newData.deltaTime, newData.imuGyro)+settings.odoMagFactor*θ_ang(posState.θ, newData.deltaTime, newData.imuGyro)) / (settings.odoGyroFactor + settings.odoMagFactor + settings.odoSteerFactor)
      ϕ₀ = (settings.odoSteerFactor*ϕ_acc+settings.odoGyroFactor*ϕ_ang(posState.ϕ, newData.deltaTime, newData.imuGyro)) / (settings.odoGyroFactor + settings.odoSteerFactor)

      return PositionalState(newPosition,
                             v,
                             (ws) ? Ψᵥₒ : (1-ratedCC)*Ψₒ + ratedCC*Ψᵥₒ, 
                             (ws) ? θᵥₒ : (1-ratedCC)*θ₀ + ratedCC*θᵥₒ,
                             (ws) ? ϕᵥₒ : (1-ratedCC)*ϕ₀ + ratedCC*ϕᵥₒ,
                             P_c_update,
                             P_g_update,
                             posState.Σ,
                             posState.Χ
                        )
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

      angle = ((northVector[2] > 0) ? π/2 : 3*π/2) - atan(northVector[1], northVector[2])

      return Float32(abs((angle > 0) ? angle : angle + 2*π))
end

"""
This method is a part of postprocessing.
"""
function smoothPoseEstimation(poses::StructVector{PositionalState}, σ::Float64, lengthInfluence::Float64)
      result = poses

      l = round(Int64, length(poses)/lengthInfluence)
      kernel = gaussian(l, σ)
      kernel = kernel ./ sum(kernel)

      # convolute position with gauss kernel
      posMatrix = reduce(vcat, transpose.(poses.position))
      posMatrix = conv(posMatrix, kernel)[Int.(l:end-l+1), :]
      
      for i ∈ Int.(ceil(l/2):length(poses)-floor(l/2))
            newState = PositionalState(result[i])

            newState.position = posMatrix[Int.(i-ceil(l/2)+1), :]
            global result[i] = newState
      end

      return result
end

"""
This function predicts position from recorded data.

# Arguments
- `posData::StructVector{PositionalData}`: The recorded data points.
- `settings::PredictionSettings`: The parameters to influence prediction quality.
"""
function predictFromRecordedData(posData::StructVector{PositionalData}, settings::PredictionSettings)
      # Set up predicted states and initialize first one
      estimatedStates = StructArray(PositionalState[])
      Ψᵢₙᵢₜ = convertMagToCompass(posData[1].imuMag)
      θᵢₙᵢₜ, ϕᵢₙᵢₜ = θ_ϕ_acc(posData[1].imuAcc, Ψᵢₙᵢₜ)
      Σᵢₙᵢₜ = Float32.(Matrix(I, n, n))
      Χᵢₙᵢₜ = settings.UKF ? generateSigmaPoints(Float32.([posData[1].cameraPos[1], posData[1].cameraPos[2], posData[1].cameraPos[3], Ψᵢₙᵢₜ, θᵢₙᵢₜ]), Σᵢₙᵢₜ, settings) : Vector{Vector{Float32}}(undef, 0)
      push!(estimatedStates, PositionalState(posData[1].cameraPos[1:3], posData[1].sensorSpeed, Ψᵢₙᵢₜ, θᵢₙᵢₜ, ϕᵢₙᵢₜ, Matrix(I, 5, 5), Matrix(I, 2, 2), Σᵢₙᵢₜ, Χᵢₙᵢₜ))
      # Predict for every coming positional value
      for i in 2:length(posData)
            push!(estimatedStates, predict(
                  estimatedStates[i-1],
                  # give mutiple positional data points if possible
                  (i > 9) ? posData[(i-9):i] : posData[(i-1):i],
                  settings, iteration=i
            ))
      end

      return estimatedStates
end

"""
Start sensor fusion with a continuous flow of data.

# Arguments
- `posData::StructVector{PositionalData}`: The last recorded data points. Length should be more than 2.
"""
function initializeSensorFusion(posData::StructVector{PositionalData}, settings::PredictionSettings)
      if length(predictedStates) == 0
            # Set first state
            Ψᵢₙᵢₜ = convertMagToCompass(posData[1].imuMag)
            θᵢₙᵢₜ, ϕᵢₙᵢₜ = θ_ϕ_acc(posData[1].imuAcc, Ψᵢₙᵢₜ)
            Σᵢₙᵢₜ = Float32.(Matrix(I, n, n))
            Χᵢₙᵢₜ = settings.UKF ? generateSigmaPoints(Float32.([posData[1].cameraPos[1], posData[1].cameraPos[2], posData[1].cameraPos[3], Ψᵢₙᵢₜ, θᵢₙᵢₜ]), Σᵢₙᵢₜ, settings) : Vector{Vector{Float32}}(undef, 0)
            global predictedStates[1] = PositionalState(posData[1].cameraPos[1:3], posData[1].sensorSpeed, Ψᵢₙᵢₜ, θᵢₙᵢₜ, ϕᵢₙᵢₜ, Matrix(I, 5, 5), Matrix(I, 2, 2), Σᵢₙᵢₜ, Χᵢₙᵢₜ)
      end

      push!(predictedStates, predict(predictedStates[length(predictedStates)  - 1], posData, settings))
end