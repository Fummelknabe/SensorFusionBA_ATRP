using GLFW


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

    GLFW.SetWindowCloseCallback(window, (_) -> onWindowClose())
    GLFW.SetMouseButtonCallback(window, (_, button, action, mods) -> onMouseButton(button, action))
    GLFW.SetKeyCallback(window, (_, key, scancode, action, mods) -> begin
        name = GLFW.GetKeyName(key, scancode)
        if name === nothing
            println("scancode $scancode ", action)
        else
            onKeyPressed(name, action)
        end
    end)
    
    GC.gc()

    return window
end

function onWindowClose()
    println("window closed")
end

function onKeyPressed(key, action)
    println(key, action)
end

function onMouseButton(button, action)
    println(button, action)
end
