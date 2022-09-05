using JSON
# Used for select file dialog
using Gtk

firstMagValue = -1

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
    posData.sensorSpeed = parse(Float32, splitted[5])
    posData.cameraPos = parse.(Float32, split(chop(splitted[6]; head=1, tail=1), ','))
    posData.cameraOri = parse.(Float32, split(chop(splitted[7]; head=1, tail=1), ','))
    posData.imuAcc = parse.(Float32, split(chop(splitted[9]; head=1, tail=1), ','))
    posData.imuGyro = parse.(Float32, split(chop(splitted[10]; head=1, tail=1), ','))
    posData.imuMag = parse.(Float32, split(chop(splitted[11]; head=1, tail=1), ','))
    posData.deltaTime = deltaTime
    posData.cameraConfidence = parse(Float32, splitted[8])

    return posData
end

function convertDictToPosData(dict::Dict, rotateCameraCoords::Bool)
    posData = PositionalData()
        
    posData.steerAngle = dict["steerAngle"]
    posData.sensorAngle = dict["sensorAngle"]
    posData.maxSpeed = dict["maxSpeed"]
    posData.sensorSpeed = dict["sensorSpeed"]
    posData.imuMag = dict["imuMag"]
    camPos = dict["cameraPos"]
    camPos = [camPos[1], -camPos[3], camPos[2], camPos[4]]
    if rotateCameraCoords 
        firstMagValue == -1 && global firstMagValue = posData.imuMag
        camPos = transformCameraCoords(Float32.(camPos), convertMagToCompass(firstMagValue)) 
    end
    posData.cameraPos = camPos
    posData.cameraOri = dict["cameraOri"]
    posData.imuGyro = deg2rad.(dict["imuGyro"])
    posData.imuAcc = dict["imuAcc"]    
    posData.deltaTime = dict["deltaTime"]
    posData.cameraConfidence = dict["cameraConfidence"] ./ 100

    return posData
end

function loadFromJSon(rotateCameraCoords::Bool)
    posData = StructArray(PositionalData[])
    filename = open_dialog("Select JSON to load")
    posDataDicts = JSON.parsefile(filename, dicttype=Dict, inttype=Int64)

    for dict in posDataDicts        
        push!(posData, convertDictToPosData(dict, rotateCameraCoords))
    end
    
    return posData
end

function loadFromJSon()
    settings = PredictionSettings(false, false, 5, false, 5, false, 0.075, 0.33, 0.66, 0, 0, 0, 0, 0, 1/3, false)
    filename = open_dialog("Select JSON to load")
    settingsDict = JSON.parsefile(filename, dicttype=Dict, inttype=Int64)

    settings.exponentCC = settingsDict["exponentCC"]
    settings.speedExponentCC = settingsDict["speedExponentCC"]
    settings.kalmanFilterCamera = settingsDict["kalmanFilterCamera"]
    settings.kalmanFilterGyro = settingsDict["kalmanFilterGyro"]
    settings.measurementNoiseC = settingsDict["measurementNoiseC"]
    settings.measurementNoiseG = settingsDict["measurementNoiseG"]
    settings.processNoiseC = settingsDict["processNoiseC"]
    settings.processNoiseG = settingsDict["processNoiseG"]
    settings.odoGyroFactor = settingsDict["odoGyroFactor"]
    settings.odoMagFactor = settingsDict["odoMagFactor"]
    settings.odoSteerFactor = settingsDict["odoSteerFactor"]
    settings.steerAngleFactor = settingsDict["steerAngleFactor"]
    settings.speedUseSinCC = settingsDict["speedSinCC"]
    settings.useSinCC = settingsDict["useSinCC"]
    settings.σ_forSpeedKernel = settingsDict["σ_forSpeedKernel"]
    settings.ΨₒmagInfluence = settingsDict["ΨₒmagInfluence"]

    return settings
end
