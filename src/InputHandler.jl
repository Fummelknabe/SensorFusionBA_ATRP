include("DataExtractor.jl")

recordData = false
automaticInput = Vector{Tuple{String, Float64}}(undef, 0)
automaticInputIndex = 1

function commandLoop(window::GLFW.Window; automaticInput::Union{String, Nothing}=nothing)
    # Key Inputs:
    #a = 65, d=68, w=87, s=83, shift =340,ctrl = 341, space=32, esc = 256
    if connected
        posData = PositionalData()

        posData.sensorSpeed = 5
        posData.steerAngle = 107

        return posData
    end
    
    command = ""

    # Escape
    if GLFW.GetKey(window, GLFW.KEY_ESCAPE)
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
    if GLFW.GetKey(window, GLFW.KEY_A)
        command *= "_left"
    elseif GLFW.GetKey(window, GLFW.KEY_D)
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
        answer = sendAndRecvData(isnothing(automaticInput) ? command : automaticInput)

        return extractData(answer)
    catch error
        @warn "Error was caught: " * string(error)
    end    
end

function createCommand(itemL::Int32, itemS::Int32, itemA::Int32)
    command = ""

    # Left and right
    if itemS == 0
        command *= "_left"
    elseif itemS == 1
        command *= "_right"
    end

    # Accelerate and Deccelerate
    if itemA == 0
        command *= "_accelerate"
    elseif itemA == 1
        command *= "_deccelerate"
    end

    # Forward, Backwards and Stop
    if itemL == 0
        command *= "_forward"
    elseif itemL == 1
        command *= "_backward"
    elseif itemL == 2
        command *= "_stop"
    else
        command *= "_nothing"
    end

    return command
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

function onWindowResize(width, height)
    glViewport(0, 0, width, height)
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