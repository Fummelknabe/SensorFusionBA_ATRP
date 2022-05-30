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
    splitted = split(data, "|")

    # if data is corrupted
    if !(length(splitted) == 8)
        println(splitted)
        println("Length was not correct: " * string(length(splitted)))
        return
        #throw(MethodError(extractData, "Missing data from AT-SV => Cannot Extract Data"))
    end

    println("Status: " * splitted[1] * "\n" *
            "Set Speed: " * splitted[2] * "\n" *
            "Steer Angle: " * splitted[3] * "\n" *
            "Detected Speed: " * splitted[4] * "\n" *
            "Camera Vector: " * splitted[5] * "\n" *
            "IMU Acceleration: " * splitted[6] * "\n" *
            "IMU Angular Rate: " * splitted[7] * "\n" *
            "IMU Magnetic Field: " * splitted[8])
end