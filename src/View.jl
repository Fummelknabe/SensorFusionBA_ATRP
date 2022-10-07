using GLFW
using CImGui
using ImPlot
using ModernGL
using CSyntax
using FileIO
using CImGui: ImVec2

# Status Text for connection Window
connectStatus = ""

# time since last frame / data
deltaTime = 0.0

include("Sensorfusion.jl") 
include("Client.jl")
include("InputHandler.jl")

const robotSource = GLTF.load("assets/robot.gltf")
const robotData = [read("assets/"*b.uri) for b in robotSource.buffers]

const plateSource = GLTF.load("assets/plate.gltf")
const plateData = [read("assets/"*b.uri) for b in plateSource.buffers]

const vertShaderScript = read("shader/shader.vert", String)
const fragShaderScript = read("shader/shader.frag", String)
estimating = false
estimation = StructArray(PositionalState[])
updateEstimation = false

estSettingWindow = false
loadingSettingsJSON = false

truePosData = Matrix{Float32}(undef, 3, 0)


export setUpWindow
"""
Set up a GLFW window, callbacks and render context.

# Arguments
- `size::Tuple{Integer, Integer}`: Size of the window.
- `title::String`: The title of the window.
"""
function setUpWindow(size::Tuple{Integer, Integer}, title::String, iconPath::String)
    window = GLFW.CreateWindow(size[1], size[2], title)    
    #icon = reinterpret(NTuple{4, UInt8}, FileIO.load(iconPath))
    #GLFW.SetWindowIcon(window,  icon)
    GLFW.MakeContextCurrent(window)
    #GLFW.PollEvents()
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
    # adjust glViewport when resizing
    GLFW.SetWindowSizeCallback(window, (window, width, height) -> onWindowResize(width, height))

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
    CImGui.SetWindowFontScale(globalFontScale)
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
    CImGui.SetWindowFontScale(globalFontScale)

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

function handleAutomaticInputWindow()
    CImGui.Begin("Automatic Inputs", C_NULL, CImGui.ImGuiWindowFlags_AlwaysAutoResize)

    @cstatic itemL=Cint(3) itemS=Cint(2) itemA=Cint(2) time=Cdouble(1.0) begin
        @c CImGui.Combo("Select longitudinal Input", &itemL, "Forward\0Backward\0Stop\0Nothing")
        @c CImGui.Combo("Select Steering Input", &itemS, "Left\0Right\0Straight")
        @c CImGui.Combo("Acceleration", &itemA, "Accelerate\0Deccelerate\0Keep")
        @c CImGui.InputDouble("Enter Time of Input [s]", &time)

        CImGui.Button("Add") && push!(automaticInput, (createCommand(itemL, itemS, itemA), time))
        CImGui.SameLine()    
        if !isempty(automaticInput) 
            CImGui.Button("Delete") && pop!(automaticInput) 
            CImGui.SameLine()
            if CImGui.Button(useAutomaticInput ? "Stop Sending!" : "Send to Robot!") 
                global useAutomaticInput = !useAutomaticInput
                global automaticInputIndex = 1
            end
        end
        CImGui.SameLine()
        ShowHelpMarker("Add Input to List of automatic inputs. Or delete last input")
    end 

    CImGui.Text("Input List:")
    for s ∈ automaticInput
        CImGui.Text(s[1]*" for $(s[2])s.")
    end

    CImGui.End()
end

function handleShowDataWindow()
    CImGui.Begin("Load Positional Data as JSON", C_NULL, CImGui.ImGuiWindowFlags_AlwaysAutoResize)
    CImGui.SetWindowFontScale(globalFontScale)
    @cstatic check=false begin 
        CImGui.Button(showRecoredDataPlots ? "Close Plots" : "Load from data") && (toggleRecordedDataPlots(showRecoredDataPlots ? StructArray(PositionalData[]) : loadDataFromJSon(rotateCameraCoords=check)))
        CImGui.SameLine()    
        @c CImGui.Checkbox("Rotate Camera Coords", &check)
        CImGui.SameLine()
        ShowHelpMarker("Unnecessary if the correct initial transform is choosen for the camera.\n This transforms the camera data onto the predicted data.")
    end
    if showRecoredDataPlots
        @cstatic  dispDataPoints=Cint(1) play=false begin
            CImGui.Text("Display Datapoint: ")
            CImGui.SameLine()
            if updateDispDataPoints dispDataPoints = Cint(rawSaveDataLength) end
            global updateEstimation = @c CImGui.SliderInt("", &dispDataPoints, 1, length(rawSavePosData), "%d")
            play || global rawSaveDataLength = dispDataPoints
            CImGui.SameLine()
            @c CImGui.Checkbox("Auto Play", &play)
            if (play && length(rawSavePosData) > rawSaveDataLength) 
                global rawSaveDataLength += 1 
                dispDataPoints = Cint(rawSaveDataLength)
            end
            CImGui.SameLine()
            ShowHelpMarker("Use Slider to display set amount of data points.")
        end
    end
end

function handleRecordDataWindow(amountDataPoints)
    CImGui.Begin("Record Positional Data", C_NULL, CImGui.ImGuiWindowFlags_AlwaysAutoResize)
    CImGui.SetWindowFontScale(globalFontScale)
    CImGui.Text(" Specify the amount of datapoints to save. \n Click 'Record' to save the next 'x' datapoints.")
    CImGui.Text("Enter Amount:")
    CImGui.SameLine()
    dataLength = 0
    CImGui.InputText("", amountDataPoints, length(amountDataPoints), CImGui.ImGuiInputTextFlags_EnterReturnsTrue) && (dataLength = toggleRecordData(amountDataPoints))
    CImGui.Button(recordData ? "Recording" : "Record") && (dataLength = toggleRecordData(amountDataPoints))  
    CImGui.End()
    return dataLength
end

function toggleRecordedDataPlots(posData::StructArray)
    global showRecoredDataPlots = isempty(posData) ? false : !showRecoredDataPlots
    global rawSavePosData = posData
end

function estimationSettingsWindow()
    CImGui.Begin("Estimation Settings")
    CImGui.SetWindowFontScale(globalFontScale)
    pred = PredictionSettings(false, false, false, 0, false, 0, false, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, false, 1.0, 1.0)

    if CImGui.Button("Toggle use Parameters in JSON")
        global loadingSettingsJSON = !loadingSettingsJSON    
        global updateEstimation = true  
        return loadingSettingsJSON ? loadParamsFromJSon() : -1
    end  

    @cstatic check=false check2=false check3=false exponent=Cfloat(5.0) useSin=false magInf=false speedExponent=Cfloat(5.0) useSinSpeed=false factor=Cfloat(1.0) steerFactor=Cfloat(0.33) gyroFactor=Cfloat(0.66) magFactor=Cfloat(0.0) r_c=Cfloat(0.1) q_c=Cfloat(0.0) r_g=Cfloat(0.1) q_g=Cfloat(0.0) r_s=Cfloat(1.0) q_s=Cfloat(0.1) σ=Cfloat(1/3) κ=Cfloat(1.0) α=Cfloat(1e-3) begin 
        updateVector = Vector{Bool}(undef, 0)
        push!(updateVector, @c CImGui.Checkbox("Kalman Filter for Camera Data", &check))
        push!(updateVector, @c CImGui.Checkbox("Kalman Filter for Gyroscope Data", &check2))
        push!(updateVector, @c CImGui.Checkbox("UKF for Kinematic Model", &check3))

        CImGui.Text("Camera Confidence Impact")
        push!(updateVector, @c CImGui.SliderFloat("##exponent", &exponent, 0.0, 40.0))
        CImGui.SameLine()
        ShowHelpMarker("At 0, camera is fully trusted.")
        CImGui.SameLine()
        push!(updateVector, @c CImGui.Checkbox("##use_sin", &useSin))
        CImGui.SameLine()
        ShowHelpMarker("Use a Sinus for Camera Confidence.")

        CImGui.Text("Camera Confidence Impact on Speed")
        push!(updateVector, @c CImGui.SliderFloat("##exponent_speed", &speedExponent, 0.0, 40.0))
        CImGui.SameLine()
        ShowHelpMarker("At 0, camera is fully trusted.")
        CImGui.SameLine()
        push!(updateVector, @c CImGui.Checkbox("##use_sin_speed", &useSinSpeed))
        CImGui.SameLine()
        ShowHelpMarker("Use a Sinus for Camera Confidence on speed.")

        CImGui.Text("Factor to adjust steerangle")
        push!(updateVector, @c CImGui.SliderFloat("##factor", &factor, 0.0, 10.0))
        CImGui.SameLine()
        ShowHelpMarker("At 0, robot always goes straight.")

        CImGui.Text("Factor to influence steering angle part.")
        push!(updateVector, @c CImGui.SliderFloat("##steer_factor", &steerFactor, 0.0, 1.0))

        CImGui.Text("Factor to influence gyroscope part.")
        push!(updateVector, @c CImGui.SliderFloat("##gyro_factor", &gyroFactor, 0.0, 1.0))

        CImGui.Text("Factor to influence compass course part.")
        push!(updateVector, @c CImGui.SliderFloat("##mag_factor", &magFactor, 0.0, 1.0))
        CImGui.SameLine()
        push!(updateVector, @c CImGui.Checkbox("##mag_influence", &magInf))
        CImGui.SameLine()
        ShowHelpMarker("Should magnetometer data influence previous state.")

        CImGui.Text("Modify Kernel to smooth speed value.")
        push!(updateVector, @c CImGui.SliderFloat("##variance", &σ, 0.01, 1.0))

        if check
            if CImGui.CollapsingHeader("Kalman Filter Settings (Camera)", C_NULL, CImGui.ImGuiTreeNodeFlags_DefaultOpen)
                CImGui.Text("Measurement Noise")
                push!(updateVector, @c CImGui.SliderFloat("##measurement_noise_c", &r_c, 0.01, 100.0))
                CImGui.Text("Process Noise")
                push!(updateVector, @c CImGui.SliderFloat("##process_noise_c", &q_c, 0.0, 0.1))
            end
        end

        if check2
            if CImGui.CollapsingHeader("Kalman Filter Settings (Gyroscope)", C_NULL, CImGui.ImGuiTreeNodeFlags_DefaultOpen)
                CImGui.Text("Measurement Noise")
                push!(updateVector, @c CImGui.SliderFloat("##measurement_noise_g", &r_g, 0.01, 100.0))
                CImGui.Text("Process Noise")
                push!(updateVector, @c CImGui.SliderFloat("##process_noise_g", &q_g, -0.1, 0.1))
            end
        end

        if check3 
            if CImGui.CollapsingHeader("Kalman Filter Settings (Kinematic Model)", C_NULL, CImGui.ImGuiTreeNodeFlags_DefaultOpen)
                CImGui.Text("Measurement Noise")
                push!(updateVector, @c CImGui.SliderFloat("##measurement_noise_s", &r_s, 0.01, 1000.0))
                CImGui.Text("Process Noise")
                push!(updateVector, @c CImGui.SliderFloat("##process_noise_s", &q_s, 0.0, 10.0))
                CImGui.Text("Kappa")
                push!(updateVector, @c CImGui.SliderFloat("##kappa", &κ, 0.0, 10.0))
                CImGui.Text("Alpha")
                push!(updateVector, @c CImGui.SliderFloat("##alpha", &α, 0.00001, 0.01))
            end
        end

        CImGui.End()

        global updateEstimation = sum(updateVector) > 0 || updateEstimation
        pred = PredictionSettings(check, check2, check3, exponent, useSin, speedExponent, useSinSpeed, factor, steerFactor, gyroFactor, magFactor, q_c, r_c, q_g, r_g, q_s, r_s, σ, magInf, κ, α)
    end 

    if loadingSettingsJSON return -1 end
    return pred
end

"""
Plot the positional data received from the AT-RP.
Has to be called inside the render loop.

# Arguments 
- `rectSize::Tuple{Integer, Integer}`: The size of the rectangle to draw position on.
- `posData::StructVector{PositionalData}`: The positional data from the atrp to plot.
- `windowName::String`: The name of the window.
"""
function plotData(rectSize::Tuple{Integer, Integer}, posData::StructVector{PositionalData}, windowName::String, settings::PredictionSettings)
    CImGui.SetNextWindowSizeConstraints(rectSize, (rectSize[1], windowSize[2]))
    CImGui.Begin(windowName, C_NULL, CImGui.ImGuiWindowFlags_AlwaysVerticalScrollbar)    
    CImGui.SetWindowFontScale(globalFontScale)

    showCameraPos = false

    # Draw the estimation button under map    
    if CImGui.Button(estimating ? "Estimating..." : "Update Estimation")
        global estimating = !estimating
        global updateEstimation = true
    end
    CImGui.SameLine()
    CImGui.Button(estSettingWindow ? "Close Settings" : "Open Settings") && global estSettingWindow = !estSettingWindow
    CImGui.SameLine()
    if CImGui.Button(length(truePosData) == 0 ? "Load True Pos Data" : "True Pos Data Loaded")
        global truePosData = loadPosFromJSon()
        # Project ground truth onto camera starting point
        global truePosData = truePosData .- ((truePosData[:, 1] - posData[1].cameraPos[1:3]) .* ones(size(truePosData)))
    end
    CImGui.SameLine()
    @cstatic showCameraPosC=false begin
    @c CImGui.Checkbox("Show Raw Camera Position", &showCameraPosC) 
    showCameraPos = showCameraPosC end

    cameraPosMatrix = reduce(vcat, transpose.(posData.cameraPos))

    # Atleast 3 data points are required to predict accurately
    if estimating
        updateVector = Vector{Bool}(undef, 0)
        if updateEstimation
            global estimation = predictFromRecordedData(posData, settings)
            push!(updateVector, updateEstimation)
            global updateEstimation = false
        end
        @cstatic smoothing=false σ=Cfloat(0.5) l=Cfloat(10.0) begin            
            CImGui.SameLine()
            push!(updateVector, @c CImGui.Checkbox("Smooth", &smoothing))      
            if smoothing            
                push!(updateVector, @c CImGui.SliderFloat("Sigma", &σ, 0.001, 20.0))
                push!(updateVector, @c CImGui.SliderFloat("Length", &l, 1.0, 100.0))
                updateEstimation = sum(updateVector) > 0
                updateEstimation && global estimation = smoothPoseEstimation(estimation, Float64(σ), Float64(l))                
            end
            CImGui.SetCursorPosY(35)
            CImGui.SameLine()
            CImGui.Button("Save To File") && saveStateToDataFile(estimation)   
            CImGui.SetCursorPosY(100)
        end
        estimationMatrix = reduce(vcat, transpose.(estimation.position))  
    end

    # Scatter plot positions 
    ImPlot.SetNextPlotLimits(-50, 50, -50, 50)   
    if ImPlot.BeginPlot("Positions", "x [m]", "y [m]", ImVec2(rectSize[1], rectSize[2]))         
        if showCameraPos
            xValues = float.(cameraPosMatrix[:, 1])
            yValues = float.(cameraPosMatrix[:, 2])
            ImPlot.PlotScatter("Camera Pos", xValues, yValues, length(posData))
        end
        if estimating             
            xValues = float.(estimationMatrix[:, 1])
            yValues = float.(estimationMatrix[:, 2])
            ImPlot.PlotScatter("Predicted Pos", xValues, yValues, length(posData))
        end
        if length(truePosData) != 0
            xValues = float.(truePosData[1, :])
            yValues = float.(truePosData[2, :])
            ImPlot.PlotScatter("Ground-Truth", xValues, yValues, Int(length(truePosData)/3))
        end
        ImPlot.EndPlot()
    end

    if CImGui.CollapsingHeader("Show Data Plots")
        if CImGui.CollapsingHeader("Estimated Position") && estimating
            estimationMatrix = reduce(vcat, transpose.(estimation.position))  
            ImPlot.SetNextPlotLimits(0, length(rawSavePosData), minimum(estimationMatrix), maximum(estimationMatrix))
            if ImPlot.BeginPlot("Predicted Position", "Data Point", "Distance [m]")
                xValues = float.(estimationMatrix[:, 1]) 
                ImPlot.PlotLine("x", xValues, size(xValues, 1))
                yValues = float.(estimationMatrix[:, 2]) 
                ImPlot.PlotLine("y", yValues, size(yValues, 1))
                zValues = float.(estimationMatrix[:, 3]) 
                ImPlot.PlotLine("z", zValues, size(zValues, 1))
                ImPlot.EndPlot()
            end
        end

        if CImGui.CollapsingHeader("Estimated Orientation") && estimating
            ImPlot.SetNextPlotLimits(0, length(rawSavePosData), -π, π)
            if ImPlot.BeginPlot("Ψ", "Data Point", "Orientation [°]")
                # Converting Ψ to compass course in degrees       
                #=      
                for i in 1:length(estimation.Ψ)
                    estimation.Ψ[i] = estimation.Ψ[i] * 180/π
                    #estimation.Ψ[i] = (estimation.Ψ[i] < 0) ? estimation.Ψ[i] + 360 : estimation.Ψ[i]
                    estimation.θ[i] = estimation.θ[i] * 180/π
                    #estimation.θ[i] = (estimation.θ[i] < 0) ? estimation.θ[i] + 360 : estimation.θ[i]
                    estimation.ϕ[i] = estimation.ϕ[i] * 180/π
                    #estimation.ϕ[i] = (estimation.ϕ[i] < 0) ? estimation.ϕ[i] + 360 : estimation.ϕ[i]
                end
                =#
                values = float.(estimation.Ψ) 
                ImPlot.PlotLine("Yaw Angle", values, size(values, 1))
                values = float.(estimation.θ) 
                ImPlot.PlotLine("Pitch Angle", values, size(values, 1))
                values = float.(estimation.ϕ) 
                ImPlot.PlotLine("Roll Angle", values, size(values, 1))
                ImPlot.EndPlot()
            end
        end

        if CImGui.CollapsingHeader("Estimated Speed") && estimating
            value = float.(estimation.v) 
            ImPlot.SetNextPlotLimits(0, length(rawSavePosData), minimum(value), maximum(value))
            if ImPlot.BeginPlot("Estimaed v", "Data Point", "v [m/s]")                
                ImPlot.PlotLine("v", value, size(value, 1))
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

        if CImGui.CollapsingHeader("Camera Orientation")
            cameraOriMatrix = reduce(vcat, transpose.(posData.cameraOri))
            ImPlot.SetNextPlotLimits(0, length(rawSavePosData), minimum(cameraOriMatrix), maximum(cameraOriMatrix))
            if ImPlot.BeginPlot("Relative Camera Orientation", "Data Point", "Degrees [°]") 
                eulerAngles = Matrix{Float32}(undef, 3, 0)
                for i ∈ 1:size(cameraOriMatrix)[1]
                    eulerAngles = hcat(eulerAngles, convertQuaternionToEuler([cameraOriMatrix[i, 1], cameraOriMatrix[i, 2], cameraOriMatrix[i, 3], cameraOriMatrix[i, 4]]))
                end
                yValues = float.(eulerAngles[2, :]) 
                ImPlot.PlotLine("Yaw", yValues, size(yValues, 1))
                yValues = float.(eulerAngles[1, :]) 
                ImPlot.PlotLine("Pitch", yValues, size(yValues, 1))
                yValues = float.(eulerAngles[3, :]) 
                ImPlot.PlotLine("Roll", yValues, size(yValues, 1))
                ImPlot.EndPlot()
            end
        end

        if CImGui.CollapsingHeader("Camera Confidence")
            ImPlot.SetNextPlotLimits(0, length(posData), 0, 1)
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
            ImPlot.SetNextPlotLimits(0, length(posData), -14, 14)
            if ImPlot.BeginPlot("Steering Angle", "Data Point", "Angle [°]")
                values = Int64.(posData.steerAngle)  
                ImPlot.PlotLine("", values, size(values, 1))
                ImPlot.EndPlot()
            end
        end

        if CImGui.CollapsingHeader("Steering Angle (Sensor)")
            ImPlot.SetNextPlotLimits(0, length(posData), -100, 100)
            if ImPlot.BeginPlot("Steering Angle (Sensor)", "Data Point", "Steering [%]")
                values = Int64.(posData.sensorAngle)  
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
    end # End CollapsingHeader

    if CImGui.CollapsingHeader("Estimation Settings")
        CImGui.Text("Use Kalman Filter Camera: $(settings.kalmanFilterCamera)")
        CImGui.Text("Use Kalman Filter Gyro: $(settings.kalmanFilterGyro)")
        CImGui.Text("Use UKF: $(settings.UKF)")
        CImGui.Text("Camera Confidence Impact: $(settings.exponentCC)")
        CImGui.Text("Use Sin for Camera: $(settings.useSinCC)")
        CImGui.Text("Camera Confidence Impact on Speed: $(settings.speedExponentCC)")
        CImGui.Text("Use Sinus for Speed: $(settings.speedUseSinCC)")
        CImGui.Text("Factor to adjust steerangle: $(settings.steerAngleFactor)")
        CImGui.Text("Factor to influence steering angle part: $(settings.odoSteerFactor)")
        CImGui.Text("Factor to influence gyroscope part: $(settings.odoGyroFactor)")
        CImGui.Text("Factor to influence compass course part: $(settings.odoMagFactor)")
        CImGui.Text("Modify Kernel to smooth speed value: $(settings.σ_forSpeedKernel)")
        CImGui.Text("Influence of magnetometer values on next State: $(settings.ΨₒmagInfluence)")
        CImGui.Text("Measurement Noise Camera: $(settings.measurementNoiseC)")
        CImGui.Text("Process Noise Camera: $(settings.processNoiseC)")
        CImGui.Text("Measurement Noise Gyro: $(settings.measurementNoiseG)")
        CImGui.Text("Process Noise Gyro: $(settings.processNoiseG)")      
        CImGui.Text("Measurement Noise UKF: $(settings.measurementNoiseS)")
        CImGui.Text("Process Noise UKF: $(settings.processNoiseS)")     
        CImGui.Text("UKF parameter kappa: $(settings.κ)") 
        CImGui.Text("UKF parameter alpha: $(settings.α)")
    end
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


