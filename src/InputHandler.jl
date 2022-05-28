

function handleKeyInputs()
    # Key Inputs:
    #a = 65, d=68, w=87, s=83, shift =340,ctrl = 341, space=32, esc = 256
    if CImGui.IsKeyPressed(65) begin println("You are Pressing: A") end end
    if CImGui.IsKeyPressed(68) begin println("You are Pressing: D") end end
    if CImGui.IsKeyPressed(87) begin println("You are Pressing: W") end end
    if CImGui.IsKeyPressed(83) begin println("You are Pressing: S") end end
    if CImGui.IsKeyPressed(340) begin println("You are Pressing: SHIFT") end end
    if CImGui.IsKeyPressed(341) begin println("You are Pressing: CTRL") end end
    if CImGui.IsKeyPressed(32) begin println("You are Pressing: SPACE") end end
    if CImGui.IsKeyPressed(256) begin println("You are Pressing: ESC") end end
end

function onMouseButton(button, action)
    #println("Mouse Button: " * string(button))
end