using CImGui
using ImPlot

using CImGui.GLFWBackend
using CImGui.OpenGLBackend
using CImGui.GLFWBackend.GLFW
using CImGui.OpenGLBackend.ModernGL

using CImGui.CSyntax
using CImGui.CSyntax.CStatic
import CImGui.LibCImGui: ImGuiCond_Once

include("View.jl")

#running = true
showHelperWindow = false;
showConnectWindow = false

function mainLoop(window::GLFW.Window, ctx)
    glClear() = ccall(@eval(GLFW.GetProcAddress("glClear")), Cvoid, (Cuint,), 0x00004000)
    
    try
        while !GLFW.WindowShouldClose(window)
            ImGui_ImplOpenGL3_NewFrame()
            ImGui_ImplGlfw_NewFrame()            
            CImGui.NewFrame()

            # Menu Bar
            begin 
                CImGui.BeginMainMenuBar()
                CImGui.MenuItem("Help") && global showHelperWindow = !showHelperWindow
                CImGui.MenuItem("Connect Window") && global showConnectWindow = !showConnectWindow
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
            
            #vec = collect(ones(100))
            #CImGui.Begin("Plot", C_NULL, CImGui.ImGuiWindowFlags_AlwaysAutoResize | CImGui.ImGuiWindowFlags_NoCollapse)
            #if ImPlot.BeginPlot("Test Data", "x", "y", CImGui.ImVec2(-1,300))
            #    ImPlot.PlotLine(vec)
            #    ImPlot.EndPlot()
            #end
            #CImGui.End()

            # if connected this call interupts for 0.05sec
            commandLoop()

            CImGui.Render()
            glClear()
            ImGui_ImplOpenGL3_RenderDrawData(CImGui.GetDrawData())

            GLFW.SwapBuffers(window)
            GLFW.WaitEvents()
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
    window, ctx = setUpWindow((1000, 800), "AT-RP Controller")
    mainLoop(window, ctx)
end

main()