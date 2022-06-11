include("DataExtractor.jl")

function commandLoop()
    # Key Inputs:
    #a = 65, d=68, w=87, s=83, shift =340,ctrl = 341, space=32, esc = 256
    if !connected
        """TESTING
        
        """
        posData = PositionalData()
        posData.steerAngle = Int(round(rand()*16+117))
        posData.maxSpeed = Int(round(rand()*21+19))+ (rand() > 0.5 ? 0.5 : 0.0)
        posData.sensorSpeed = rand() * 15
        posData.imuMag = rand() * 360
        posData.cameraPos = [-0.5, 1, 2*rand(), rand() * 0.1 * (rand() > 0.5 ? -1 : 1)]
        posData.imuAcc = [rand(), rand(), rand()]
        posData.imuGyro = [rand(), rand(), rand()]
        return posData#return 0
    end
    
    command = ""

    sleep(0.05)

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
    if CImGui.IsKeyPressed(65) 
        command *= "_left"
    elseif CImGui.IsKeyPressed(68)
        command *= "_right"
    end

    # Accelerate and Deccelerate
    if CImGui.IsKeyPressed(340) 
        command *= "_accelerate"
    elseif CImGui.IsKeyPressed(341) 
        command *= "_deccelerate"
    end

    # Forward, Backwards and Stop
    if CImGui.IsKeyPressed(87) 
        command *= "_forward"
    elseif CImGui.IsKeyPressed(83) 
        command *= "_backward"
    elseif CImGui.IsKeyPressed(32)
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
        println("Disconnecting...")
        answer = sendAndRecvData("escape")
        if answer == "closing" 
            global connected = false
            global connectStatus = ""
            pysocket.close()
        end
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