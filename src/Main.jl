using CImGui.GLFWBackend
using CImGui.OpenGLBackend
using CImGui.GLFWBackend.GLFW
using CImGui.OpenGLBackend.ModernGL

include("Client.jl")
include("View.jl")

running = true

function mainLoop(window::GLFW.Window, ctx)
    glClear() = ccall(@eval(GLFW.GetProcAddress("glClear")), Cvoid, (Cuint,), 0x00004000)

    try
        while !GLFW.WindowShouldClose(window)
            ImGui_ImplOpenGL3_NewFrame()
            ImGui_ImplGlfw_NewFrame()            
            CImGui.NewFrame()

            # Window
            begin
                # Create a window
                CImGui.Begin("Hello, world!")
                CImGui.Text("This is some useful text.")
                CImGui.End()
            end            

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

"""
This is the starting point of the program.
"""
function main()
    #println("The program is started! \nTrying to connect to Jetson:")
    #
    #if connectToJetson(HOST, PORT)
    #    println("Connection established!")
    #end
    #
    ## Send a few commands for testing purposes:
    #println(sendAndRecvData("Hallo Jetson!"))

    # Create window and start main loop
    window, ctx = setUpWindow((600, 400), "AT-RP Controller")
    mainLoop(window, ctx)
end

main()