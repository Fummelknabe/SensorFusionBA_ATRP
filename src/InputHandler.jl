include("DataExtractor.jl")

function commandLoop()
    # Key Inputs:
    #a = 65, d=68, w=87, s=83, shift =340,ctrl = 341, space=32, esc = 256
    if !connected
        return
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
            return
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

        extractData(answer)
    catch error
        @warn "Error was caught: " * string(error)
    end    
end

function onMouseButton(button, action)
    #println("Mouse Button: " * string(button))
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