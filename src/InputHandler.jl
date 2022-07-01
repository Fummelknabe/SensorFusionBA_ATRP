include("DataExtractor.jl")

recordData = false

function commandLoop(window::GLFW.Window)
    # Key Inputs:
    #a = 65, d=68, w=87, s=83, shift =340,ctrl = 341, space=32, esc = 256
    if !connected
        #=
        posData = PositionalData()
        posData.steerAngle = Int(round(rand()*16+117))
        posData.maxSpeed = Int(round(rand()*21+19))+ (rand() > 0.5 ? 0.5 : 0.0)
        posData.sensorSpeed = rand() * 15
        posData.imuMag = rand() * 360
        posData.cameraPos = [-0.5, 1, 2*rand(), rand() * 0.1 * (rand() > 0.5 ? -1 : 1)]
        posData.imuAcc = [rand(), rand(), rand()]
        posData.imuGyro = [rand(), rand(), rand()]
        return posData =#
        return 0
    end
    
    command = ""

    # Escape
    if CImGui.IsKeyPressed(256) 
        @info "Sending Escape Command and Disconnect."
        answer = sendAndRecvData("escape")

        if answer == "closing"
            global connected = false
            global connectStatus = ""
            pysocket.close()
            return 0
        end
    end

    # Left and right
    if GLFW.GetKey(window, GLFW.KEY_LEFT)
        command *= "_left"
    elseif GLFW.GetKey(window, GLFW.KEY_RIGHT)
        command *= "_right"
    end

    # Accelerate and Deccelerate
    if GLFW.GetKey(window, GLFW.KEY_LEFT_SHIFT)
        command *= "_accelerate"
    elseif GLFW.GetKey(window, GLFW.KEY_LEFT_CONTROL)
        command *= "_deccelerate"
    end

    # Forward, Backwards and Stop
    if GLFW.GetKey(window, GLFW.KEY_W)
        command *= "_forward"
    elseif GLFW.GetKey(window, GLFW.KEY_S)
        command *= "_backward"
    elseif GLFW.GetKey(window, GLFW.KEY_SPACE)
        command *= "_stop"
    else
        command *= "_nothing"
    end    

    try
        answer = sendAndRecvData(command)

        return extractData(answer)
    catch error
        @warn "Error was caught: " * string(error)
    end    
end

function onMouseButton(button, action)
    if button == GLFW.MOUSE_BUTTON_LEFT && action == GLFW.PRESS
        global isLeftMouseButtonDown = true       
    elseif  button == GLFW.MOUSE_BUTTON_LEFT && action == GLFW.RELEASE
        global isLeftMouseButtonDown = false
    end
    
    if button == GLFW.MOUSE_BUTTON_RIGHT && action == GLFW.PRESS
        global isRightMouseButtonDown = true       
    elseif  button == GLFW.MOUSE_BUTTON_RIGHT && action == GLFW.RELEASE
        global isRightMouseButtonDown = false
    end
end

function connectButtonPress(ipData::String, portData::String)
    if connected
        disconnect()
        return
    end

    ip = ""
    port = ""

    for char in ipData
        if char === '.' || isdigit(char)
            ip = ip * char
        end
    end

    for char in portData
        if isdigit(char)
            port = port * char
        end
    end

    global connectStatus = "Trying to connect to: " * string(ip) * " on " * string(port)
    global connectStatus = checkConnection(ip, port)
end

function toggleRecordData(amountDataPoints::String)
    if recordData 
        global recordData = false 

        if size(rawSavePosData, 1) > 0 
            open("data/pos_data.json", "w") do ioStream
                JSON.print(ioStream, rawSavePosData, 4)
            end            
        end 
        return 0
    end

    amount = ""
    for char in amountDataPoints
        if isdigit(char)
            amount = amount * char
        end
    end

    if length(amount) < 1
        @warn "Amount to Record is not valid."
        return 0
    end

    dataLength = parse(Int64, amount)
    if dataLength == 0
        @warn "Amount to Record is not valid."
        return 0
    end

    global recordData = true
    return dataLength
end