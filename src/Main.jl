using CImGui

using CImGui.GLFWBackend
using CImGui.OpenGLBackend
using CImGui.GLFWBackend.GLFW
using CImGui.OpenGLBackend.ModernGL

using CImGui.CSyntax
using CImGui.CSyntax.CStatic

using StructArrays
using LinearAlgebra

include("Structs.jl")

# How many positional data points to save
const rawDataLength = 100
rawSaveDataLength = 1

# Booleans for the open windows
showHelperWindow = false
showConnectWindow = false
showDataPlots = false
renderRobot = false
recordDataWindow = false
showRecoredDataPlots = false
showLoadDataWindow = false

isLeftMouseButtonDown = false
isRightMouseButtonDown = false
oldMousePosition = [0.0, 0.0]
windowSize = (1600, 900)

include("Camera.jl")
# Camera to render
cam = Camera()
cam.position = GLfloat[-4.0, -4.0, 3.0]

const l_f = 0.59
const l_r = 0.10

include("Model.jl")
include("View.jl")

# Raw Data
rawPositionalData = StructArray(PositionalData[])
# Raw Data to save
rawSavePosData = StructArray(PositionalData[])

function mainLoop(window::GLFW.Window, ctx, program) 
    models = [loadGLTFModelInBuffers(robotSource, robotData), loadGLTFModelInBuffers(plateSource, plateData)]
    models[1].transform.scale = [0.15, 0.15, 0.15]
    models[1].transform.position = [0.0, 2.5, 0.0]
    models[1].transform.eulerRotation = [0.0, -π/8, π]
    saveDataLength = 0

    try
        while !GLFW.WindowShouldClose(window)
            updateFPS(window)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            ImGui_ImplOpenGL3_NewFrame()
            ImGui_ImplGlfw_NewFrame()            
            CImGui.NewFrame()

            if renderRobot
                # Before calculatin camera matrices check Inputs
                checkCameraMovement([CImGui.GetMousePos().x - windowSize[1] / 2, CImGui.GetMousePos().y - windowSize[2] / 2], cam)
                if connected
                    # if connected orientate robot according to camera orientation in x and z axis
                    positionalData = rawPositionalData[length(rawPositionalData)]
                    angles = convertQuaternionToEuler(positionalData.cameraOri) 
                    models[1].transform.eulerRotation = [angles[1], -π/8, π + angles[3]]

                    # Get the meshes of the robot model
                    leftWheel = models[1].meshes[2]                    
                    rightWheel = models[1].meshes[4]                    
                    rearAxis = models[1].meshes[3]

                    # change transform according to sensor values
                    leftWheel.transform.eulerRotation = [0.0, 0.0, (positionalData.steerAngle - 120) * π/180]
                    rightWheel.transform.eulerRotation = [0.0, 0.0, (positionalData.steerAngle - 120) * π/180]
                    rearAxis.transform.eulerRotation = [0.0, rearAxis.transform.eulerRotation[2] + positionalData.sensorSpeed * 5, 0.0]
                end

                for model in models                
                    
                    modelTransform = transformToMatrix(model.transform)
                    for mesh in model.meshes                    
                        writeToUniforms(program, modelTransform * transformToMatrix(mesh.transform), cam, GLfloat[1.0, 1.0, 1.0], mesh.material)

                        glBindVertexArray(mesh.vao)
                        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo)
                        glDrawElements(GL_TRIANGLES, mesh.sizeOfIndices, GL_UNSIGNED_INT - 2, Ptr{Cvoid}(0))
                        # unbind
                        glBindVertexArray(0)
                    end
                end
            end

            # CImGui Stuff:
            # Menu Bar
            begin 
                CImGui.BeginMainMenuBar()
                CImGui.MenuItem("Help") && global showHelperWindow = !showHelperWindow
                CImGui.MenuItem("Connect Window") && global showConnectWindow = !showConnectWindow
                CImGui.MenuItem("Data Plots") && global showDataPlots = !showDataPlots
                CImGui.MenuItem("Render Robot") && global renderRobot = !renderRobot
                CImGui.MenuItem("Record Data") && global recordDataWindow = !recordDataWindow
                CImGui.MenuItem("Load Data") && global showLoadDataWindow = !showLoadDataWindow
                CImGui.EndMainMenuBar()
            end

            # Helper Window             
            showHelperWindow && handleHelperWidow()

            # Connection Window  
            if showConnectWindow          
            @cstatic portData = ""*"\0"^115 i0=Cint(123) @cstatic ipData = ""*"\0"^115 i0=Cint(123) begin
                    handleConnectWindow(ipData, portData)
                end  
            end   

            # Record Data Window
            if recordDataWindow
            @cstatic amountDataPoints = ""*"\0"^115 i0=Cint(123) begin
                    !recordData && (saveDataLength = handleRecordDataWindow(amountDataPoints))
                    recordData && handleRecordDataWindow(amountDataPoints)
                end
            end

            if showLoadDataWindow
                handleShowDataWindow()
            end
 
            # if connected this call interupts for 0.025sec
            posData = commandLoop(window)

            # Add Positional Data to storage
            if posData != 0 && !isnothing(posData)
                push!(rawPositionalData, posData)
                if size(rawPositionalData, 1) > rawDataLength
                    popfirst!(rawPositionalData)
                end

                if recordData
                    push!(rawSavePosData, posData)
                    if size(rawSavePosData, 1) > saveDataLength
                        toggleRecordData("")
                        global rawSavePosData = StructArray(PositionalData[])
                    end
                end
            end

            # initialize with standard settings
            settings = PredictionSettings(false, false, 5, false, 5, false, 0.075, 0.33, 0.66, 0, 0, 0, 0, 0, 1/3)
            if predSettingWindow
                settings = predictionSettingsWindow()
            end

            if showRecoredDataPlots
                plotData((1000, 700), rawSavePosData[1:rawSaveDataLength], "Recorded data Plot", settings)
            end

            if showDataPlots && size(rawPositionalData, 1) > 0
                plotData((1000, 700), rawPositionalData, "On time positional data", settings)
            end            

            CImGui.Render()
            ImGui_ImplOpenGL3_RenderDrawData(CImGui.GetDrawData())                 

            GLFW.SwapBuffers(window)
            GLFW.PollEvents()
        end
    finally
        CImGui.DestroyContext(ctx)
        GLFW.DestroyWindow(window)
    end
end

"""
This is the starting point of the program.
"""
function main()
    # Create window and start main loop
    window, ctx, program = setUpWindow(windowSize, "AT-RP Controller", "assets/icon_64.png")
    cam.aspectRatio = windowSize[1]/windowSize[2]
    mainLoop(window, ctx, program)
end

main()