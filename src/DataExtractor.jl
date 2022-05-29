export PositionalData
"""
Struct to store the data acquired by the AT-RP
"""
struct PositionalData
    steerAngle::Integer
    maxSpeed::Integer
    sensorSpeed::Float32
    cameraPos::Vector{Float32}
    imuGyro::Vector{Float32}
    imuAcc::Vector{Float32}
    imuMag::Vector{Float32}
end

export extractData
"""
This method extracts the data send by the AT-RP and 
stores them into the PositionalData Type.

# Arguments
- `data::String`: The data as a string with structure: \n
<repetition of command>-<status>-<speed value>-<steering angle>-<detected speed in m/s>-<camera vector>-<imu data>.
"""
function extractData(data::String)
    splitted = split(data, "-")

    # if data is corrupted
    if !length(splitted) == 7
        throw(MethodError(extractData, "Missing data from AT-SV => Cannot Extract Data"))
    end

    println("Command: " * splitted[1] * "\n" *
            "Status: " * splitted[2] * "\n" *
            "Set Speed: " * splitted[3] * "\n" *
            "Steer Angle: " * splitted[4] * "\n" *
            "Detected Speed: " * splitted[5] * "\n" *
            "Camera Vector: " * splitted[6] * "\n" *
            "IMU Data: " * splitted[7])
end