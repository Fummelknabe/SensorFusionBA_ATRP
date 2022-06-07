using CImGui

using CImGui.GLFWBackend
using CImGui.OpenGLBackend
using CImGui.GLFWBackend.GLFW
using CImGui.OpenGLBackend.ModernGL

using CImGui.CSyntax
using CImGui.CSyntax.CStatic

using StructArrays

# How many positional data points to save
const rawDataLength = 100

#running = true
showHelperWindow = false
showConnectWindow = false
showDataPlots = false

include("View.jl")

# Raw Data
rawPositionalData = StructArray(PositionalData[])

function mainLoop(window::GLFW.Window, ctx, vao, program)
    #glClear() = ccall(@eval(GLFW.GetProcAddress("glClear")), Cvoid, (Cuint,), 0x00004000)
    
    try
        while !GLFW.WindowShouldClose(window)
            #glClear()
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            ImGui_ImplOpenGL3_NewFrame()
            ImGui_ImplGlfw_NewFrame()            
            CImGui.NewFrame()

            glUseProgram(program)
            glBindVertexArray(vao)
            glDrawArrays(GL_TRIANGLES, 0, 3)

            # Menu Bar
            begin 
                CImGui.BeginMainMenuBar()
                CImGui.MenuItem("Help") && global showHelperWindow = !showHelperWindow
                CImGui.MenuItem("Connect Window") && global showConnectWindow = !showConnectWindow
                CImGui.MenuItem("Data Plots") && global showDataPlots = !showDataPlots
                CImGui.EndMainMenuBar()
            end

            # Helper Window             
            if showHelperWindow
                handleHelperWidow()
            end

            # Connection Window  
            if showConnectWindow          
            @cstatic portData = ""*"\0"^115 i0=Cint(123) @cstatic ipData = ""*"\0"^115 i0=Cint(123) begin
                    handleConnectWindow(portData, ipData)
                end  
            end      
 
            # if connected this call interupts for 0.05sec
            posData = commandLoop()

            # Add Positional Data to storage
            if posData != 0
                push!(rawPositionalData, posData)
                if size(rawPositionalData, 1) > rawDataLength
                    popfirst!(rawPositionalData)
                end
            end

            if showDataPlots
                plotRawData(rawPositionalData)
            end

            CImGui.Render()
            ImGui_ImplOpenGL3_RenderDrawData(CImGui.GetDrawData())
                 

            GLFW.SwapBuffers(window)
            GLFW.WaitEvents(0.01)
        end
    finally
        CImGui.DestroyContext(ctx)
        GLFW.DestroyWindow(window)
    end
end

function inputTextCallback()
    println("Im here")
end

"""
This is the starting point of the program.
"""
function main()
    # Create window and start main loop
    window, ctx, vao, program = setUpWindow((1400, 1000), "AT-RP Controller")
    mainLoop(window, ctx, vao, program)
end

main()