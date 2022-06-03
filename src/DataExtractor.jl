using JSON

export PositionalData
"""
Struct to store the data acquired by the AT-RP
"""
mutable struct PositionalData
    steerAngle::Integer
    maxSpeed::Float32
    sensorSpeed::Float32
    cameraPos::Vector{Float32}
    imuGyro::Vector{Float32}
    imuAcc::Vector{Float32}
    imuMag::Float32

    PositionalData() = new(0, 0.0, 0.0, [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0)
end

export extractData
"""
This method extracts the data send by the AT-RP and 
stores them into the PositionalData Type.

# Arguments
- `data::String`: The data as a string with structure: \n
<repetition of command>-<status>-<speed value>-<steering angle>-<detected speed in m/s>-<camera vector>-<imu data>.

# Returns 
- `PositionalData`: All the positional data combined in one datatype.
"""
function extractData(data::String)
    splitted = split(data, "|")

    # if data is corrupted
    if !(length(splitted) == 8)
        println(splitted)
        println("Length was not correct: " * string(length(splitted)))
        return
    end

    posData = PositionalData()
    posData.maxSpeed = parse(Float32, splitted[2])
    posData.steerAngle = parse(Int, splitted[3])
    posData.sensorSpeed = parse(Float32, splitted[4])
    posData.cameraPos = parse.(Float32, split(chop(splitted[5]; head=1, tail=1), ','))
    posData.imuAcc = parse.(Float32, split(chop(splitted[6]; head=1, tail=1), ','))
    posData.imuGyro = parse.(Float32, split(chop(splitted[7]; head=1, tail=1), ','))
    posData.imuMag = parse(Float32, splitted[8])

    return posData
end