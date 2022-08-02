using GLFW
using CImGui
using ImPlot
using ModernGL
using CSyntax
using CImGui: ImVec2

# Status Text for connection Window
connectStatus = ""

# time since last frame / data
deltaTime = 0.0

include("Sensorfusion.jl")
include("Client.jl")
include("InputHandler.jl")

#const robotModelSource = GLTF.load("assets/simpleTank.gltf")
#const robotModelData = [read("assets/"*b.uri) for b in robotModelSource.buffers]

const robotSource = GLTF.load("assets/robot.gltf")
const robotData = [read("assets/"*b.uri) for b in robotSource.buffers]

const plateSource = GLTF.load("assets/plate.gltf")
const plateData = [read("assets/"*b.uri) for b in plateSource.buffers]

const vertShaderScript = read("shader/shader.vert", String)
const fragShaderScript = read("shader/shader.frag", String)

predicting = false
prediction = StructArray(PositionalState[])

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

function ShowHelpMarker(description)
    CImGui.TextDisabled("(?)")
    if CImGui.IsItemHovered()
        CImGui.BeginTooltip()
        CImGui.PushTextWrapPos(CImGui.GetFontSize() * 35.0)
        CImGui.TextUnformatted(description)
        CImGui.PopTextWrapPos()
        CImGui.EndTooltip()
    end
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
    if showRecoredDataPlots
        @cstatic  dispDataPoints=Cint(1) begin
            CImGui.Text("Display Datapoint: ")
            CImGui.SameLine()
            @c CImGui.SliderInt("", &dispDataPoints, 1, length(rawSavePosData), "%d")
            global rawSaveDataLength = dispDataPoints
            CImGui.SameLine()
            ShowHelpMarker("Use Slider to display set amount of data points.")
        end
    end
    CImGui.End()
    return dataLength
end

function toggleRecordedDataPlots(posData::StructArray)
    global showRecoredDataPlots = !showRecoredDataPlots
    if showRecoredDataPlots
        # Transform camera position but this doenst work yet
        global rawSavePosData = posData
    else 
        global rawSavePosData = StructArray(PositionalData[])
    end
end

"""
Plot the positional data received from the AT-RP.
Has to be called inside the render loop.

# Arguments 
- `rectSize::Tuple{Integer, Integer}`: The size of the rectangle to draw position on.
- `posData::StructVector{PositionalData}`: The positional data from the atrp to plot.
- `windowName::String`: The name of the window.
"""
function plotData(rectSize::Tuple{Integer, Integer}, posData::StructVector{PositionalData}, windowName::String)
    CImGui.SetNextWindowSizeConstraints(rectSize, (rectSize[1], windowSize[2]))
    CImGui.Begin(windowName, C_NULL, CImGui.ImGuiWindowFlags_AlwaysVerticalScrollbar)
    
    # Plot camera pos
    drawList = CImGui.GetWindowDrawList()    
    rectPos = CImGui.GetWindowPos()

    cameraPosMatrix = reduce(vcat, transpose.(posData.cameraPos))#rawSavePosData for keeping same size of rectangle

    CImGui.SetCursorPos(Float32.((0.0, rectSize[2])))

    # Draw the prediction button under map
    CImGui.Button(predicting ? "Predicting..." : "Update Prediction") && global predicting = !predicting

    if predicting
        startPosState = PositionalState([cameraPosMatrix[1, 1], cameraPosMatrix[1, 3], cameraPosMatrix[1, 2]], [posData.sensorSpeed[1], posData.sensorSpeed[1]], 0.0)
        global prediction = initializeSensorFusion(startPosState, posData)
    end

    CImGui.SetCursorPos((0, 20))
    CImGui.AddRectFilled(drawList, rectPos, (rectPos.x + rectSize[1], rectPos.y + rectSize[2] - CImGui.GetScrollY()), CImGui.IM_COL32(100, 100, 100, 255))
    
    (minX, minZ) = (minimum(float.(cameraPosMatrix[:, 1])), minimum(float.(cameraPosMatrix[:, 3])))
    (maxX, maxZ) = (maximum(float.(cameraPosMatrix[:, 1])), maximum(float.(cameraPosMatrix[:, 3])))

    if predicting
        predictionMatrix = reduce(vcat, transpose.(prediction.position))   

        (predMinX, predMinY) = (minimum(float.(predictionMatrix[:, 1])), minimum(float.(predictionMatrix[:, 2])))
        (predMaxX, predMaxY) = (maximum(float.(predictionMatrix[:, 1])), maximum(float.(predictionMatrix[:, 2])))

        (minX, minZ) = (minimum([predMinX, minX]), minimum([predMinY, minZ]))
        (maxX, maxZ) = (maximum([predMaxX, maxX]), maximum([predMaxY, maxZ]))
    end

    xDif = abs(minX) + abs(maxX)
    zDif = abs(minZ) + abs(maxZ)
    factor = round(minimum((rectSize[1] / xDif, rectSize[2] / zDif)))    # Pixel pro meter

    meanX = round((minX + maxX) / 2)
    meanZ = round((minZ + maxZ) / 2)

    for i in 1:length(posData.cameraPos)
        lastPoint = i == length(posData.cameraPos)
        pointPos = (rectPos.x + floor(rectSize[1]/2) + (posData.cameraPos[i][1] - meanX)*factor, rectPos.y + floor(rectSize[2]/2) - CImGui.GetScrollY() + (posData.cameraPos[i][3] - meanZ)*factor) # factor * 10
        CImGui.AddCircleFilled(drawList, pointPos, lastPoint ? 5 : 1, 
            lastPoint ? CImGui.IM_COL32(0, 255, 0, 255) : CImGui.IM_COL32(255, 0, 0, 255))

        if predicting
            predPointPos = (rectPos.x + floor(rectSize[1]/2) + (prediction.position[i][1] - meanX)*factor, rectPos.y + floor(rectSize[2]/2) - CImGui.GetScrollY() + (prediction.position[i][2] - meanZ)*factor) # factor * 10                       
            CImGui.AddCircleFilled(drawList, predPointPos, lastPoint ? 5 : 1, 
                lastPoint ? CImGui.IM_COL32(0, 0, 255, 255) : CImGui.IM_COL32(60, 130, 60, 255))
        end
    end

    # Spacing to accomodate for rect
    CImGui.Dummy(0.0, rectSize[2])

    if CImGui.CollapsingHeader("Prediction Position") && predicting
        predictionMatrix = reduce(vcat, transpose.(prediction.position))  
        ImPlot.SetNextPlotLimits(0, length(rawSavePosData), minimum(predictionMatrix), maximum(predictionMatrix))
        if ImPlot.BeginPlot("Predicted Position", "Data Point", "Distance [m]")
            xValues = float.(predictionMatrix[:, 1]) 
            ImPlot.PlotLine("x", xValues, size(xValues, 1))
            yValues = float.(predictionMatrix[:, 2]) 
            ImPlot.PlotLine("y", yValues, size(yValues, 1))
            zValues = float.(predictionMatrix[:, 3]) 
            ImPlot.PlotLine("z", zValues, size(zValues, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Prediction Ψ") && predicting
        ImPlot.SetNextPlotLimits(0, length(rawSavePosData), minimum(prediction.Ψ), maximum(prediction.Ψ))
        if ImPlot.BeginPlot("Ψ", "Data Point", "Orientation [°]")
            values = float.(prediction.Ψ) 
            ImPlot.PlotLine("Ψ", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end

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

    if CImGui.CollapsingHeader("Camera Confidence")
        ImPlot.SetNextPlotLimits(0, length(posData), 0, 100)
        if ImPlot.BeginPlot("Confidence Value for Camera", "Data Point", "Percent [%]")
            values = float.(posData.cameraConfidence) 
            ImPlot.PlotLine("", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Camera Position Change")
        camPosChange = float.(cameraPosMatrix[:, 4])
        ImPlot.SetNextPlotLimits(0, length(posData), minimum(camPosChange), maximum(camPosChange))
        if ImPlot.BeginPlot("Positional Change", "Data Point", "Absolute Change")
            ImPlot.PlotLine("", camPosChange, size(camPosChange, 1))
            ImPlot.EndPlot()
        end
    end
    if CImGui.CollapsingHeader("Steering Angle")
        ImPlot.SetNextPlotLimits(0, length(posData), -20, 20)
        if ImPlot.BeginPlot("Steering Angle", "Data Point", "Angle [°]")
            values = Int64.(posData.steerAngle)  
            ImPlot.PlotLine("", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Max Speed")
        ImPlot.SetNextPlotLimits(0, length(posData), 19, 40)
        if ImPlot.BeginPlot("Max Speed", "Data Point", "Max Speed [PWM - Duty Cycle]")
            values = float.(posData.maxSpeed)  
            ImPlot.PlotLine("", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end
    
    if CImGui.CollapsingHeader("Speed")
        values = float.(posData.sensorSpeed)
        ImPlot.SetNextPlotLimits(0, length(posData), minimum(values), maximum(values))
        if ImPlot.BeginPlot("Speed", "Data Point", "Speed [m/s]")             
            ImPlot.PlotLine("", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Magnetometer")
        # Convert vector of vectors to matrix:
        imuMagMatrix = reduce(vcat, transpose.(posData.imuMag))
        ImPlot.SetNextPlotLimits(0, length(posData), -1, 1)
        if ImPlot.BeginPlot("Magnetic Field", "Data Point", "Field Strength [G]")
            yValues = float.(imuMagMatrix[:, 1]) 
            ImPlot.PlotLine("x", yValues, size(yValues, 1))
            yValues = float.(imuMagMatrix[:, 2]) 
            ImPlot.PlotLine("y", yValues, size(yValues, 1))
            yValues = float.(imuMagMatrix[:, 3]) 
            ImPlot.PlotLine("z", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Angular Velocity")
        # Convert vector of vectors to matrix:
        imuGyroMatrix = reduce(vcat, transpose.(posData.imuGyro))
        ImPlot.SetNextPlotLimits(0, length(posData), minimum(imuGyroMatrix), maximum(imuGyroMatrix))
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
        ImPlot.SetNextPlotLimits(0, length(posData), minimum(imuAccMatrix), maximum(imuAccMatrix))
        if ImPlot.BeginPlot("Acceleration", "Data Point", "Distance [g]")            
            yValues = float.(imuAccMatrix[:, 1]) 
            ImPlot.PlotLine("x", yValues, size(yValues, 1))
            yValues = float.(imuAccMatrix[:, 2]) 
            ImPlot.PlotLine("y", yValues, size(yValues, 1))
            yValues = float.(imuAccMatrix[:, 3]) 
            ImPlot.PlotLine("z", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Delta Time")
        values = float.(posData.deltaTime)
        ImPlot.SetNextPlotLimits(0, length(posData), minimum(values), maximum(values))
        if ImPlot.BeginPlot("Delta Time", "Data Point", "dt [s]")             
            ImPlot.PlotLine("", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end

    CImGui.End()
end

#=
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

    #=
    Disable plotting this as its subject to change
    if CImGui.CollapsingHeader("Compass Course")
        ImPlot.SetNextPlotLimits(0, rawDataLength, 0, 360)
        if ImPlot.BeginPlot("Angle to Magnetic North", "Data Point", "Degrees [°]")
            yValues = float.(posData.imuMag) 
            ImPlot.PlotLine("", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end
    =#

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
        if ImPlot.BeginPlot("Acceleration", "Data Point", "Distance [g]")            
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
=#

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


