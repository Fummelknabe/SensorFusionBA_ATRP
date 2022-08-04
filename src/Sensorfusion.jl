using DSP
using LinearAlgebra

rateCameraConfidence(cc) = cc^5

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

function computeSpeed(cameraChange::Vector{Float32}, δt::Vector{Float32}, v::Float32, camConfidence::Float32)
      length(cameraChange) == length(δt) || throw("Computing Speed failed, as $(cameraChange) and $(δt) were not the same length.")
      
      # Filter speed from camera change
      l = length(cameraChange)
      kernel = gaussian(l, 1/3)
      kernel = kernel ./ sum(kernel)
      cameraSpeed = conv(cameraChange ./ δt, kernel)
      
      # Combine with wheel odometry speed value
      cc = rateCameraConfidence(camConfidence)
      return (1-cc) * v + cc * cameraSpeed[l]
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
Angle between velocity vector and axis thought the wheels of the vehicle
"""
function β(δ) return atan(l_r / (l_r + l_f) * tan(δ)) end

"""
The angle between the x axis and the axis through the wheels of the vehicle
"""
Ψ(Ψ_, δt, δ, v) = Ψ_ + v / (l_r + l_f) * cos(β(δ)) * tan(δ) * δt 

v_x(v_x_, β, δt, Ψ, a_x) = (v_x_ + a_x*δt) * cos(Ψ + β) 
v_y(v_y_, β, δt, Ψ, a_x) = (v_y_ + a_x*δt) * sin(Ψ + β) 

p_x(p_x_, v_x, δt, Ψ, β, a_x) = p_x_ + v_x * δt + 1/2 * a_x * cos(Ψ + β) * δt^2 
p_y(p_y_, v_y, δt, Ψ, β, a_x) = p_y_ + v_y * δt + 1/2 * a_x * sin(Ψ + β) * δt^2 
# Z Position is not always the same, but we dont easy data to calculate it 
# transform position p after calculating in x-y plane
p_z(p_z_) = p_z_ 


function predict(posState::PositionalState, δt::Float32, steeringAngle::Integer, acceleration::Vector{Float32}, velocityForward::Float32, Ψ_sensor::Float32)
      oldPos = posState.position
      oldVel = velocityForward#posState.velocity
      oldΨ = Ψ_sensor #* π/180#posState.Ψ
      a_x = acceleration[1] * 9.81

      β_ = β(steeringAngle * π/180)
      Ψ_ = Ψ(oldΨ, δt, steeringAngle, norm(oldVel))
      velocity = [v_x(oldVel, β_, δt, Ψ_, a_x), v_y(oldVel, β_, δt, Ψ_, a_x)]

      return PositionalState(
                  [p_x(oldPos[1], velocity[1], δt, Ψ_, β_, a_x), p_y(oldPos[2], velocity[2], δt, Ψ_, β_, a_x), p_z(oldPos[3])], 
                  velocity, 
                  Ψ_)      
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

function initializeSensorFusion(startPosState::PositionalState, posData::StructArray)
      predictedStates = StructArray(PositionalState[])

      for i in 1:length(posData)
            push!(predictedStates, predict(i == 1 ? startPosState : predictedStates[i-1], 
                                                                    posData.deltaTime[i], 
                                                                    posData.steerAngle[i], 
                                                                    posData.imuAcc[i], 
                                                                    posData.sensorSpeed[i], 
                                                                    convertMagToCompass(posData.imuMag[i]#=, accelerometerVector=posData.imuAcc[i]=#)))
      end

      return predictedStates
end