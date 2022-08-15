using JSON

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
    if !(length(splitted) == 11)
        println(splitted)
        println("Length was not correct: " * string(length(splitted)))
        return
    end

    posData = PositionalData()
    posData.maxSpeed = parse(Float32, splitted[2])
    posData.steerAngle = parse(Int, splitted[3])
    posData.sensorAngle = parse(Int, splitted[4])
    # speed value is in m/0.25s converting speed:
    posData.sensorSpeed = 4 * parse(Float32, splitted[5])
    posData.cameraPos = parse.(Float32, split(chop(splitted[6]; head=1, tail=1), ','))
    posData.cameraOri = parse.(Float32, split(chop(splitted[7]; head=1, tail=1), ','))
    posData.imuAcc = parse.(Float32, split(chop(splitted[9]; head=1, tail=1), ','))
    posData.imuGyro = parse.(Float32, split(chop(splitted[10]; head=1, tail=1), ','))
    posData.imuMag = parse.(Float32, split(chop(splitted[11]; head=1, tail=1), ','))
    posData.deltaTime = deltaTime
    posData.cameraConfidence = parse(Float32, splitted[8])

    return posData
end

function convertDictToPosData(dict::Dict)
    posData = PositionalData()
        
    posData.steerAngle = dict["steerAngle"]
    posData.sensorAngle = dict["sensorAngle"]
    posData.maxSpeed = dict["maxSpeed"]
    posData.sensorSpeed = dict["sensorSpeed"]
    camPos = dict["cameraPos"]
    posData.cameraPos = [camPos[1], -camPos[3], camPos[2], camPos[4]]
    posData.cameraOri = dict["cameraOri"]
    posData.imuGyro = deg2rad.(dict["imuGyro"])
    posData.imuAcc = dict["imuAcc"]
    posData.imuMag = dict["imuMag"]
    posData.deltaTime = dict["deltaTime"]
    posData.cameraConfidence = dict["cameraConfidence"] ./ 100

    return posData
end

function loadFromJSon()
    posData = StructArray(PositionalData[])
    posDataDicts = JSON.parsefile("data/pos_data.json", dicttype=Dict, inttype=Int64)

    for dict in posDataDicts        
        push!(posData, convertDictToPosData(dict))
    end
    
    return posData
end
