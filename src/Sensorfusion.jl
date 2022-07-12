mutable struct PositionalState
      position::Vector{Float32}
      velocity::Vector{Float32}
      Ψ::Float32
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
Compute the steering angle from all information that is available
"""
function approximateDelta(steerAngle, compassCourse, angularVelZ, )
      # if we where to compute δ with bicycle model we would get:
      # δ = atan((-l_f*Ψ*sqrt(v^2 - Ψ^2 l_r^2) - Ψ*l_r*sqrt(v^2 - Ψ^2*l_r^2))/(Ψ^2*l_r^2 - v^2))
      # this is not advisiable as we wont use more data to predict our position more accurately

      # ASK GEORG
end

"""
Angle between velocity vector and axis thought the wheels of the vehicle
"""
β(δ) = atan(l_r / (l_r + l_f) * tan(δ))

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


function predict(posState::PositionalState, δt::Float32, steeringAngle::Float32, acceleration::Vector{Float32})
      oldPos = posState.position
      oldVel = posState.velocity
      oldΨ = posState.Ψ
      a_x = acceleration[1] * 9.81

      β = β(steeringAngle)
      Ψ = Ψ(oldΨ, δt, steeringAngle, norm(oldVel))
      velocity = [v_x(oldVel[1], β, δt, Ψ, a_x), v_y(oldVel[2], β, δt, Ψ, a_x)]

      return PositionalState(
                  [p_x(oldPos[1], velocity[1], δt, Ψ, β, a_x), p_y(oldPos[2], velocity[2], δt, Ψ, β, a_x), p_z(oldPos[3])], 
                  velocity, 
                  Ψ)      
end 

function initializeSensorFusion(startPosState::PositinalState, posData::StructArray(PositionalData[]))
      predictedStates = StructArray(PositinalState[])

      for i in 1:length(posData)
            push!(predictedStates, predict(i == 1 ? startPosState : predictedStates[i-1], posData.deltaTime[i], posData.steeringAngle[i], posData.imuAcc[i]))
      end

      return predictedStates
end