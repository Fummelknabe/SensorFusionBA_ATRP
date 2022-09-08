mutable struct PositionalState
    position::Vector{Float32}
    v::Float32    
    Ψ::Float32
    θ::Float32
    P_c::Matrix{Float32}
    P_g::Matrix{Float32}
    Σ::Matrix{Float32}
    Χ::Vector{Vector{Float32}}
end

mutable struct PredictionSettings
    kalmanFilterCamera::Bool
    kalmanFilterGyro::Bool
    UKF::Bool
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
    processNoiseS::Float32
    measurementNoiseS::Float32
    σ_forSpeedKernel::Float32
    ΨₒmagInfluence::Bool
    λ::Float32
    n::Int
    α::Float32
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