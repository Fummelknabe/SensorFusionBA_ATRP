using GLFW
using CImGui
using ImPlot
using ModernGL
using CSyntax
using CImGui: ImVec2

# Status Text for connection Window
connectStatus = ""

include("Client.jl")
include("InputHandler.jl")

const robotModelSource = GLTF.load("assets/simpleTank.gltf")
const robotModelData = [read("assets/"*b.uri) for b in robotModelSource.buffers]

const vertShaderScript = read("shader/shader.vert", String)
const fragShaderScript = read("shader/shader.frag", String)

deltaTime = 0.0

export setUpWindow
"""
Set up a GLFW window, callbacks and render context.

# Arguments
- `size::Tuple{Integer, Integer}`: Size of the window.
- `title::String`: The title of the window.
"""
function setUpWindow(size::Tuple{Integer, Integer}, title::String)
    window = GLFW.CreateWindow(size[1], size[2], title)
    GLFW.MakeContextCurrent(window)
    ctx = CImGui.CreateContext()

    # Create ImPlot context
    ctxp = ImPlot.CreateContext()
    ImPlot.SetImGuiContext(ctx)

    # Load fonts and select style....
    CImGui.StyleColorsDark()

    CImGui.ImGui_ImplGlfw_InitForOpenGL(window, true)
    CImGui.ImGui_ImplOpenGL3_Init(410) # GLSL Version

    GLFW.SetWindowCloseCallback(window, (_) -> onWindowClose())
    GLFW.SetMouseButtonCallback(window, (_, button, action, mods) -> onMouseButton(button, action))

    #enable depth test 
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)

    program = createShaders()

    GC.gc()

    return window, ctx, program
end

function createShaders()
    # compile shaders
    vertShader = glCreateShader(GL_VERTEX_SHADER)
    glShaderSource(vertShader, 1, Ptr{GLchar}[pointer(vertShaderScript)], C_NULL)
    glCompileShader(vertShader)
    fragShader = glCreateShader(GL_FRAGMENT_SHADER)
    glShaderSource(fragShader, 1, Ptr{GLchar}[pointer(fragShaderScript)], C_NULL)
    glCompileShader(fragShader)

    # create and link shader program
    program = glCreateProgram()
    glAttachShader(program, vertShader)
    glAttachShader(program, fragShader)
    glLinkProgram(program)

    # enable face culling
    glEnable(GL_CULL_FACE)
    glCullFace(GL_FRONT)
    glFrontFace(GL_CW)

    # set background color to gray
    glClearColor(0.2, 0.2, 0.2, 1.0)    

    return program
end

function handleHelperWidow()
    CImGui.SetNextWindowPos((0, 20))
    CImGui.Begin("Help", C_NULL, CImGui.ImGuiWindowFlags_AlwaysAutoResize)
    CImGui.ShowUserGuide()
    CImGui.Text("Tip: Double Click on Plots to recenter")
    CImGui.Text("Robot Control:")
    CImGui.Text("
    \"W\" - Accelerate Forward \n
    \"S\" - Accelerate Backward \n 
    \"A\" - Increase Steering Left \n
    \"D\" - Increase Steering Right \n
    \"SPACE\" - Stop Motor \n
    \"Shift\" - Increase Max Speed \n
    \"Crtl\" - Decrease Max Speed
    ")
    CImGui.End()
end

function handleConnectWindow(ipData, portData)
    # Create a window
    CImGui.Begin("Connect to Jetson", C_NULL, CImGui.ImGuiWindowFlags_AlwaysAutoResize | CImGui.ImGuiWindowFlags_NoCollapse)

    CImGui.Text("Please Enter the IP Adress and Port for the Jetson")

    CImGui.Text("Enter IP:")
    CImGui.SameLine()
    CImGui.InputText("", ipData, length(ipData), CImGui.ImGuiInputTextFlags_EnterReturnsTrue) && connectButtonPress(ipData, portData)         
    CImGui.Text("Enter Port:")
    CImGui.SameLine()
    CImGui.InputText(" ", portData, length(portData), CImGui.ImGuiInputTextFlags_EnterReturnsTrue) && connectButtonPress(ipData, portData)                        
    CImGui.Button(connected == false ? "Connect" : "Disconnect") && connectButtonPress(ipData, portData)
    CImGui.Text(connectStatus)

    CImGui.End()
end

function handleRecordDataWindow(amountDataPoints)
    CImGui.Begin("Record Positional Data", C_NULL, CImGui.ImGuiWindowFlags_AlwaysAutoResize)
    CImGui.Text(" Specify the amount of datapoints to save. \n Click 'Record' to save the next 'x' datapoints.")
    CImGui.Text("Enter Amount:")
    CImGui.SameLine()
    dataLength = 0
    CImGui.InputText("", amountDataPoints, length(amountDataPoints), CImGui.ImGuiInputTextFlags_EnterReturnsTrue) && (dataLength = toggleRecordData(amountDataPoints))
    CImGui.Button(recordData ? "Recording" : "Record") && (dataLength = toggleRecordData(amountDataPoints))
    CImGui.SameLine()
    CImGui.Button(showRecoredDataPlots ? "Close Plots" : "Load from data") && (toggleRecordedDataPlots(loadFromJSon()))
    CImGui.End()
    return dataLength
end

function toggleRecordedDataPlots(posData::StructArray)
    global showRecoredDataPlots = !showRecoredDataPlots
    if showRecoredDataPlots
        global rawSavePosData = posData
    else 
        global rawSavePosData = StructArray(PositionalData[])
    end
end

function plotRecordedData(rectSize::Tuple{Integer, Integer})
    CImGui.SetNextWindowSizeConstraints(rectSize, (rectSize[1], windowSize[2]))
    CImGui.Begin("Plots of Recorded Data", C_NULL, CImGui.ImGuiWindowFlags_AlwaysVerticalScrollbar)

    # Plot camera pos
    drawList = CImGui.GetWindowDrawList()    
    rectPos = CImGui.GetWindowPos()

    CImGui.AddRectFilled(drawList, rectPos, (rectPos.x + rectSize[1], rectPos.y + rectSize[2] - CImGui.GetScrollY()), CImGui.IM_COL32(100, 100, 100, 255))
    cameraPosMatrix = reduce(vcat, transpose.(rawSavePosData.cameraPos))
    (minX, minZ) = (minimum(float.(cameraPosMatrix[:, 1])), minimum(float.(cameraPosMatrix[:, 3])))
    (maxX, maxZ) = (maximum(float.(cameraPosMatrix[:, 1])), maximum(float.(cameraPosMatrix[:, 3])))

    xDif = abs(minX) + abs(maxX)
    zDif = abs(minZ) + abs(maxZ)
    factor = round(minimum((rectSize[1] / xDif, rectSize[2] / zDif)))    

    meanX = round((minX + maxX) / 2)
    meanZ = round((minZ + maxZ) / 2)

    for camPos in rawSavePosData.cameraPos
        pointPos = (rectPos.x + floor(rectSize[1]/2) + (camPos[1] - meanX)*10*factor, rectPos.y + floor(rectSize[2]/2) - CImGui.GetScrollY() + (camPos[3] - meanZ)*10*factor)
        CImGui.AddCircleFilled(drawList, pointPos, 1, CImGui.IM_COL32(255, 0, 0, 255))
    end

    # Spacing to accomodate for rect
    CImGui.Dummy(0.0, rectSize[2])

    if CImGui.CollapsingHeader("Camera Position")
        ImPlot.SetNextPlotLimits(0, length(rawSavePosData), minimum(cameraPosMatrix), maximum(cameraPosMatrix))
        if ImPlot.BeginPlot("Relative Camera Position", "Data Point", "Distance [m]")            
            yValues = float.(cameraPosMatrix[:, 1]) 
            ImPlot.PlotLine("x", yValues, size(yValues, 1))
            yValues = float.(cameraPosMatrix[:, 2]) 
            ImPlot.PlotLine("y", yValues, size(yValues, 1))
            yValues = float.(cameraPosMatrix[:, 3]) 
            ImPlot.PlotLine("z", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Camera Position Change")
        camPosChange = float.(cameraPosMatrix[:, 4])
        ImPlot.SetNextPlotLimits(0, length(rawSavePosData), minimum(camPosChange), maximum(camPosChange))
        if ImPlot.BeginPlot("Positional Change", "Data Point", "Absolute Change")
            ImPlot.PlotLine("", camPosChange, size(camPosChange, 1))
            ImPlot.EndPlot()
        end
    end
    if CImGui.CollapsingHeader("Steering Angle")
        ImPlot.SetNextPlotLimits(0, length(rawSavePosData), -20, 20)
        if ImPlot.BeginPlot("Steering Angle", "Data Point", "Angle [°]")
            values = Int64.(rawSavePosData.steerAngle)  
            ImPlot.PlotLine("", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Max Speed")
        ImPlot.SetNextPlotLimits(0, length(rawSavePosData), 19, 40)
        if ImPlot.BeginPlot("Max Speed", "Data Point", "Max Speed [PWM - Duty Cycle]")
            values = float.(rawSavePosData.maxSpeed)  
            ImPlot.PlotLine("", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end
    
    if CImGui.CollapsingHeader("Speed")
        values = float.(rawSavePosData.sensorSpeed)
        ImPlot.SetNextPlotLimits(0, length(rawSavePosData), minimum(values), maximum(values))
        if ImPlot.BeginPlot("Speed", "Data Point", "Speed [m/s]")             
            ImPlot.PlotLine("", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Compass Course")
        ImPlot.SetNextPlotLimits(0, length(rawSavePosData), 0, 360)
        if ImPlot.BeginPlot("Angle to Magnetic North", "Data Point", "Degrees [°]")
            values = float.(rawSavePosData.imuMag) 
            ImPlot.PlotLine("", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Angular Velocity")
        # Convert vector of vectors to matrix:
        imuGyroMatrix = reduce(vcat, transpose.(rawSavePosData.imuGyro))
        ImPlot.SetNextPlotLimits(0, length(rawSavePosData), minimum(imuGyroMatrix), maximum(imuGyroMatrix))
        if ImPlot.BeginPlot("Angular Velocity", "Data Point", "Distance [°/s]")            
            yValues = float.(imuGyroMatrix[:, 1]) 
            ImPlot.PlotLine("x", yValues, size(yValues, 1))
            yValues = float.(imuGyroMatrix[:, 2]) 
            ImPlot.PlotLine("y", yValues, size(yValues, 1))
            yValues = float.(imuGyroMatrix[:, 3]) 
            ImPlot.PlotLine("z", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end 
    end

    if CImGui.CollapsingHeader("Acceleration")
        # Convert vector of vectors to matrix:
        imuAccMatrix = reduce(vcat, transpose.(rawSavePosData.imuAcc))
        ImPlot.SetNextPlotLimits(0, length(rawSavePosData), minimum(imuAccMatrix), maximum(imuAccMatrix))
        if ImPlot.BeginPlot("Acceleration", "Data Point", "Distance [m/s^2]")            
            yValues = float.(imuAccMatrix[:, 1]) 
            ImPlot.PlotLine("x", yValues, size(yValues, 1))
            yValues = float.(imuAccMatrix[:, 2]) 
            ImPlot.PlotLine("y", yValues, size(yValues, 1))
            yValues = float.(imuAccMatrix[:, 3]) 
            ImPlot.PlotLine("z", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end

    CImGui.End()
end

"""
Plot the positional data received from the AT-RP.
Has to be called inside the render loop.

# Arguments 
- `posData::StructVector{PositionalData}`: The positional data from the atrp to plot.
"""
function plotRawData(posData::StructVector{PositionalData})      
    #CImGui.SetNextWindowSize((600, 900))      
    #=This in not ideal, cause plots might not be visible with a constraint window size=#   
    CImGui.Begin("Plots", C_NULL, CImGui.ImGuiWindowFlags_AlwaysVerticalScrollbar)

    if size(posData, 1) == 0
        @info "No PositionalData to Plot"
        global showDataPlots = false
        CImGui.End()
        return
    end

    if CImGui.CollapsingHeader("Steering Angle")
        ImPlot.SetNextPlotLimits(0, rawDataLength, -20, 20)
        if ImPlot.BeginPlot("Steering Angle", "Data Point", "Angle [°]")
            yValues = Int64.(posData.steerAngle)  
            ImPlot.PlotLine("", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Max Speed")
        ImPlot.SetNextPlotLimits(0, rawDataLength, 19, 40)
        if ImPlot.BeginPlot("Max Speed", "Data Point", "Max Speed [PWM - Duty Cycle]")
            yValues = float.(posData.maxSpeed)  
            ImPlot.PlotLine("", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end
    
    if CImGui.CollapsingHeader("Speed")
        ImPlot.SetNextPlotLimits(0, rawDataLength, 0, 15)
        if ImPlot.BeginPlot("Speed", "Data Point", "Speed [m/s]")
            yValues = float.(posData.sensorSpeed) 
            ImPlot.PlotLine("", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Compass Course")
        ImPlot.SetNextPlotLimits(0, rawDataLength, 0, 360)
        if ImPlot.BeginPlot("Angle to Magnetic North", "Data Point", "Degrees [°]")
            yValues = float.(posData.imuMag) 
            ImPlot.PlotLine("", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Camera Position Change")
        ImPlot.SetNextPlotLimits(0, rawDataLength, -0.2, 0.2)
        if ImPlot.BeginPlot("Positional Change", "Data Point", "Absolute Change")
            # Convert vector of vectors to matrix:
            cameraPosMatrix = reduce(vcat, transpose.(posData.cameraPos))
            yValues = float.(cameraPosMatrix[:, 4]) 
            ImPlot.PlotLine("", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Camera Position")
        # Convert vector of vectors to matrix:
        cameraPosMatrix = reduce(vcat, transpose.(posData.cameraPos))
        ImPlot.SetNextPlotLimits(0, rawDataLength, minimum(cameraPosMatrix), maximum(cameraPosMatrix))
        if ImPlot.BeginPlot("Relative Camera Position", "Data Point", "Distance [m]")            
            yValues = float.(cameraPosMatrix[:, 1]) 
            ImPlot.PlotLine("x", yValues, size(yValues, 1))
            yValues = float.(cameraPosMatrix[:, 2]) 
            ImPlot.PlotLine("y", yValues, size(yValues, 1))
            yValues = float.(cameraPosMatrix[:, 3]) 
            ImPlot.PlotLine("z", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Angular Velocity")
        # Convert vector of vectors to matrix:
        imuGyroMatrix = reduce(vcat, transpose.(posData.imuGyro))
        ImPlot.SetNextPlotLimits(0, rawDataLength, minimum(imuGyroMatrix), maximum(imuGyroMatrix))
        if ImPlot.BeginPlot("Angular Velocity", "Data Point", "Distance [°/s]")            
            yValues = float.(imuGyroMatrix[:, 1]) 
            ImPlot.PlotLine("x", yValues, size(yValues, 1))
            yValues = float.(imuGyroMatrix[:, 2]) 
            ImPlot.PlotLine("y", yValues, size(yValues, 1))
            yValues = float.(imuGyroMatrix[:, 3]) 
            ImPlot.PlotLine("z", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end 
    end

    if CImGui.CollapsingHeader("Acceleration")
        # Convert vector of vectors to matrix:
        imuAccMatrix = reduce(vcat, transpose.(posData.imuAcc))
        ImPlot.SetNextPlotLimits(0, rawDataLength, minimum(imuAccMatrix), maximum(imuAccMatrix))
        if ImPlot.BeginPlot("Acceleration", "Data Point", "Distance [m/s^2]")            
            yValues = float.(imuAccMatrix[:, 1]) 
            ImPlot.PlotLine("x", yValues, size(yValues, 1))
            yValues = float.(imuAccMatrix[:, 2]) 
            ImPlot.PlotLine("y", yValues, size(yValues, 1))
            yValues = float.(imuAccMatrix[:, 3]) 
            ImPlot.PlotLine("z", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end

    CImGui.End()
end

let (previousTime, previousTimeCounting) = (time(), time())
    frame = 0
    global function updateFPS(window::GLFW.Window)
        currentTime = time()
        countingTime = currentTime - previousTimeCounting
        global deltaTime = currentTime - previousTime

        # update display every 0.25sec
        if countingTime > 0.25
            previousTimeCounting = currentTime
            fps = frame / countingTime
            GLFW.SetWindowTitle(window, "AT-RP Controller | FPS: $fps | dt: $deltaTime")
            frame = 0
        end
        previousTime = currentTime
        frame += 1
    end
end

function onWindowClose()
    @info "Window Closed"
end


