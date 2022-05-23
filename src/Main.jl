include("Client.jl")
include("View.jl")

running = true

function mainLoop(window::GLFW.Window)
    glClear() = ccall(@eval(GLFW.GetProcAddress("glClear")), Cvoid, (Cuint,), 0x00004000)

    try
        while !GLFW.WindowShouldClose(window)
            glClear()
            GLFW.SwapBuffers(window)
            GLFW.WaitEvents()
        end
    finally
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
    window = setUpWindow((600, 400), "AT-RP Controller")
    mainLoop(window)
end

main()