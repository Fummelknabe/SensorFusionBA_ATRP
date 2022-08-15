export PositionalState

mutable struct PositionalState
    position::Vector{Float32}
    v::Float32
    P::Matrix{Float32}
    Ψ::Float32
    θ::Float32
end

mutable struct PredictionSettings
    kalmanFilterCamera::Bool
    exponentCC::Float32
    steerAngleFactor::Float32
    steerGyroRatio::Float32
    processNoise::Float32
    measurementNoise::Float32
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

    PositionalData() = new(0, 0, 0.0, 0.0, [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0, 0.0)
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