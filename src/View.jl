using GLFW
using CImGui
using ImPlot

# Status Text for connection Window
connectStatus = ""

include("Client.jl")
include("InputHandler.jl")

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
    CImGui.ImGui_ImplOpenGL3_Init(130) # GLSL Version

    GLFW.SetWindowCloseCallback(window, (_) -> onWindowClose())
    GLFW.SetMouseButtonCallback(window, (_, button, action, mods) -> onMouseButton(button, action))

    GC.gc()

    return window, ctx
end

function handleHelperWidow()
    CImGui.SetNextWindowPos((0, 20))
    CImGui.Begin("Help", C_NULL, CImGui.ImGuiWindowFlags_AlwaysAutoResize)
    CImGui.ShowUserGuide()
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
    CImGui.InputText("", ipData, length(ipData), CImGui.ImGuiInputTextFlags_EnterReturnsTrue) && inputTextCallback()                   
    CImGui.Text("Enter Port:")
    CImGui.SameLine()
    CImGui.InputText(" ", portData, length(portData), CImGui.ImGuiInputTextFlags_EnterReturnsTrue) && inputTextCallback()                             
    CImGui.Button(connected == false ? "Connect" : "Disconnect") && connectButtonPress(ipData, portData)
    CImGui.Text(connectStatus)

    CImGui.End()
end

"""
Plot the positional data received from the AT-RP.
Has to be called inside the render loop.

# Arguments 
- `posData::StructVector{PositionalData}`: The positional data from the atrp to plot.
"""
function plotRawData(posData::StructVector{PositionalData})      
    CImGui.SetNextWindowSize((600, 900))         
    CImGui.Begin("Plots", C_NULL, CImGui.ImGuiWindowFlags_AlwaysVerticalScrollbar)

    if size(posData, 1) == 0
        @info "No PositionalData to Plot"
        global showDataPlots = false
        CImGui.End()
        return
    end

    if CImGui.CollapsingHeader("Steering Angle")
        ImPlot.SetNextPlotLimits(0, rawDataLength, 117, 133)
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

    if CImGui.CollapsingHeader("Test")
        ImPlot.SetNextPlotLimits(0, rawDataLength, 0, 360)
        if ImPlot.BeginPlot("Test Plot", "Data Point", "Degrees [°]")
            yValues = float.(posData.imuMag) 
            ImPlot.PlotScatter("", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end

    CImGui.End()
end

function onWindowClose()
    println("window closed")
end


