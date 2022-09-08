mutable struct State
    x::Float32
    y::Float32
    Ψ::Float32

    State(s::Vector{Float32}) = new(s[1], s[2], s[3])
    State(x, y, Ψ) = new(x, y, Ψ)
end

mutable struct Input
    v::Float32
    δt::Float32
    δ::Float32
end

mutable struct PositionalState
    position::Vector{Float32}
    v::Float32
    P_c::Matrix{Float32}
    P_g::Matrix{Float32}
    Ψ::Float32
    θ::Float32

    PositionalState(position, v, P_c, P_g, Ψ, θ) = new(position, v, P_c, P_g, Ψ, θ)
    PositionalState(s::State, z, v, P_c, P_g, θ) = new([s.x, s.y, z], v, P_c, P_g, s.Ψ, θ)
end

mutable struct PredictionSettings
    kalmanFilterCamera::Bool
    kalmanFilterGyro::Bool
    exponentCC::Float32
    useSinCC::Bool
    speedExponentCC::Float32
    speedUseSinCC::Bool
    steerAngleFactor::Float32
    odoSteerFactor::Float32
    odoGyroFactor::Float32
    odoMagFactor::Float32
    processNoiseC::Float32
    measurementNoiseC::Float32
    processNoiseG::Float32
    measurementNoiseG::Float32
    σ_forSpeedKernel::Float32
    ΨₒmagInfluence::Bool
    useKinematicBM::Bool
end

export PositionalData
"""
Struct to store the data acquired by the AT-RP
"""
mutable struct PositionalData
    steerAngle::Integer
    sensorAngle::Integer
    maxSpeed::Float32
    sensorSpeed::Float32
    cameraPos::Vector{Float32}
    cameraOri::Vector{Float32}    
    imuGyro::Vector{Float32}
    imuAcc::Vector{Float32}
    imuMag::Vector{Float32}
    deltaTime::Float32
    cameraConfidence::Float32
    command::String

    PositionalData() = new(0, 0, 0.0, 0.0, [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0, 0.0, String(""))
end

mutable struct Camera
    speed::Float32
    position::Vector{GLfloat}
    target::Vector{Float32}
    up::Vector{Float32}
    scrollSpeed::Float32
    aspectRatio::Float32
    fov::Int

    function Camera()
        new(0.2, GLfloat[0.0, 0.0, 2.0], [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 10.0, 16/9, 30)
    end
end