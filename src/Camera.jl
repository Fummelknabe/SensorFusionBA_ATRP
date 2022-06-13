
# distance to front and back plane
const znear = 0.1
const zfar = 100

mutable struct Camera
    speed::Float32
    position::Vector{GLfloat}
    yaw::Float32
    yawSpeed::Float32
    aspectRatio::Float32
    fov::Int

    function Camera()
        new(0.01, GLfloat[0.0, 0.0, 2.0], 0.0, 10.0, 16/9, 30)
    end
end

function getProjectionMatrix(cam::Camera)
    range = tan(0.5*cam.fov) * znear
    return GLfloat[(2.0*znear / (range*cam.aspectRatio + range*cam.aspectRatio)) 0.0 0.0 0.0;
                    0.0 (znear / range) 0.0 0.0;
                    0.0 0.0 (-(zfar+znear) / (zfar-znear)) (-(2.0*zfar*znear) / (zfar-znear));
                    0.0 0.0 -1.0 0.0]
end

function getViewMatrix(cam::Camera)
    translationalPart = GLfloat[1.0 0.0 0.0 -cam.position[1];
                         0.0 1.0 0.0 -cam.position[2];
                         0.0 0.0 1.0 -cam.position[3];
                         0.0 0.0 0.0 1.0;]
    rotationPart = GLfloat[cos(-cam.yaw) 0.0 sin(-cam.yaw) 0.0;
                            0.0 1.0 0.0 0.0;
                            -sin(-cam.yaw) 0.0 cos(-cam.yaw) 0.0;
                            0.0 0.0 0.0 1.0]

    return rotationPart * translationalPart
end

function checkCameraMovement()
    if isLeftMouseButtonDown
        cam.position[3] += GLfloat(cam.speed) 
    end

    if isRightMouseButtonDown
        cam.position[3] -= GLfloat(cam.speed) 
    end 
    # Left and right
    if CImGui.IsKeyPressed(65) 
        cam.position[1] -= GLfloat(cam.speed) 
    elseif CImGui.IsKeyPressed(68)
        cam.position[1] += GLfloat(cam.speed) 
    end
end