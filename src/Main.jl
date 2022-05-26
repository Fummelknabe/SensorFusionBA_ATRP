using CImGui

using CImGui.GLFWBackend
using CImGui.OpenGLBackend
using CImGui.GLFWBackend.GLFW
using CImGui.OpenGLBackend.ModernGL

using CImGui.CSyntax
using CImGui.CSyntax.CStatic

# Status Text for connection Window
connectStatus = ""

include("Client.jl")
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

function buttonPress(ipData::String, portData::String)
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

    global connectStatus = "Trying to connect to: " * ip * " on " * port
    global connectStatus = checkConnection(ip, port)
end

function inputTextCallback()
    println("Im here")
end

"""
This is the starting point of the program.
"""
function main()
    # Create window and start main loop
    window, ctx = setUpWindow((600, 400), "AT-RP Controller")
    mainLoop(window, ctx)
end

main()