using JSON
# Used for select file dialog
using Gtk

# The first value of the magnetometer
firstMagValue = -1
# need to update the dispay of data points value because reloaded data
updateDispDataPoints = false 

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
    posData.command = splitted[1] == "_nothing" ? String("") : splitted[1]
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
        
    posData.steerAngle = dict["steerAngle"] - 120
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
    posData.command = dict["command"]

    return posData
end

function loadDataFromJSon(;rotateCameraCoords::Bool=true)   
    posData = StructArray(PositionalData[])
    filename = open_dialog("Select JSON to load")
    if filename == "" return posData end
    @info "Loading raw data..."
    global rawSaveDataLength = 1    # reset display of pos data
    global updateDispDataPoints = true
    posDataDicts = JSON.parsefile(filename, dicttype=Dict, inttype=Int64)
    try        
        for dict in posDataDicts        
            push!(posData, convertDictToPosData(dict, rotateCameraCoords))
        end
    catch e
        if e isa LoadError
            @warn "Wrong file type. Please select the right file. Reload file."
        end
    end    
    
    return posData
end

function loadPosFromJSon()    
    posData = Matrix{Float32}(undef, 3, 0)
    filename = open_dialog("Select JSON to load")
    if filename == "" return posData end
    try
        @info "Loading pos data..."
        posDataDicts = JSON.parsefile(filename, dicttype=Dict, inttype=Int64);

        for dict in posDataDicts        
            posData = hcat(posData, [dict["x"], dict["y"], dict["z"]]);
        end
    catch e
        if e isa SystemError
            @warn "The given file does not exist."
        else
            println(e)
        end
    end

    return posData
end

function loadParamsFromJSon()
    params = PredictionSettings(false, false, false, 5, false, 5, false, 1.0, 0.33, 0.66, 0, 0, 0, 0, 0, 0, 0, 1/3, false, 1.0, 1.0)
    filename = open_dialog("Select JSON to load")
    if filename == "" 
        @warn "No File was selected."
        return params 
    end
    paramsDict = JSON.parsefile(filename, dicttype=Dict, inttype=Int64)

    params.exponentCC = paramsDict["exponentCC"]
    params.speedExponentCC = paramsDict["speedExponentCC"]
    params.kalmanFilterCamera = paramsDict["kalmanFilterCamera"]
    params.kalmanFilterGyro = paramsDict["kalmanFilterGyro"]
    params.UKF = paramsDict["UKF"]
    params.measurementNoiseC = paramsDict["measurementNoiseC"]
    params.measurementNoiseG = paramsDict["measurementNoiseG"]
    params.measurementNoiseS = paramsDict["measurementNoiseS"]
    params.processNoiseC = paramsDict["processNoiseC"]
    params.processNoiseG = paramsDict["processNoiseG"]
    params.processNoiseS = paramsDict["processNoiseS"]
    params.odoGyroFactor = paramsDict["odoGyroFactor"]
    params.odoMagFactor = paramsDict["odoMagFactor"]
    params.odoSteerFactor = paramsDict["odoSteerFactor"]
    params.steerAngleFactor = paramsDict["steerAngleFactor"]
    params.speedUseSinCC = paramsDict["speedUseSinCC"]
    params.useSinCC = paramsDict["useSinCC"]
    params.σ_forSpeedKernel = paramsDict["σ_forSpeedKernel"]
    params.ΨₒmagInfluence = paramsDict["ΨₒmagInfluence"]
    params.κ = paramsDict["κ"]
    params.α = paramsDict["α"]

    return params
end

function saveStateToDataFile(states::StructVector{PositionalState})
    open("pos_state.data", "w") do file 
        for i ∈ eachindex(states)
            if i == 1
                s = String("i positionX positionY positionZ v Psi Theta Phi")
                write(file, s*"\n");
            end
            
            s = String("")

            state = states[i]
            s = s*string(i)*" "
            s = s*string(state.position[1])*" "
            s = s*string(state.position[2])*" "
            s = s*string(state.position[3])*" "
            s = s*string(state.v)*" "
            s = s*string(state.Ψ)*" "
            s = s*string(state.θ)*" "
            s = s*string(state.ϕ)*" "

            write(file, s*"\n")
        end
    end
end


