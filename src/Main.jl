using CImGui

using CImGui.GLFWBackend
using CImGui.OpenGLBackend
using CImGui.GLFWBackend.GLFW
using CImGui.OpenGLBackend.ModernGL

using CImGui.CSyntax
using CImGui.CSyntax.CStatic

using StructArrays
using LinearAlgebra

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

isLeftMouseButtonDown = false
isRightMouseButtonDown = false
oldMousePosition = [0.0, 0.0]
windowSize = (1400, 1000)

include("Camera.jl")
# Camera to render
cam = Camera()
cam.position = GLfloat[-4.0, -4.0, 3.0]

include("Model.jl")
include("View.jl")

# Raw Data
rawPositionalData = StructArray(PositionalData[])
# Raw Data to save
rawSavePosData = StructArray(PositionalData[])

function mainLoop(window::GLFW.Window, ctx, program) 
    models = [loadGLTFModelInBuffers(robotModelSource, robotModelData)]
    models[1].transform.scale = [0.5, 0.5, 0.5]
    models[1].transform.eulerRotation = [0.0, 1/2*π, π]
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

            if showRecoredDataPlots
                plotRecordedData((1000, 700), rawSavePosData[1:rawSaveDataLength])
            end

            if showDataPlots && size(rawPositionalData, 1) > 0
                plotRawData(rawPositionalData)
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
    window, ctx, program = setUpWindow(windowSize, "AT-RP Controller")
    cam.aspectRatio = windowSize[1]/windowSize[2]
    mainLoop(window, ctx, program)
end

main()