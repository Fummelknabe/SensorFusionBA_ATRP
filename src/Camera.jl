
# distance to front and back plane
const znear = 0.1
const zfar = 100

function getProjectionMatrix(cam::Camera)
    range = tan(0.5*cam.fov) * znear
    return GLfloat[(2.0*znear / (range*cam.aspectRatio + range*cam.aspectRatio)) 0.0 0.0 0.0;
                    0.0 (znear / range) 0.0 0.0;
                    0.0 0.0 (-(zfar+znear) / (zfar-znear)) (-(2.0*zfar*znear) / (zfar-znear));
                    0.0 0.0 -1.0 0.0]
end

function getViewMatrix(cam::Camera)
    zAxis = normalize(cam.target - cam.position)
    xAxis = normalize(cross(zAxis, cam.up))
    yAxis = cross(xAxis, zAxis)

    zAxis = -1 * zAxis

    viewMatrix = GLfloat[xAxis[1] xAxis[2] xAxis[3] -dot(xAxis, cam.position);
                  yAxis[1] yAxis[2] yAxis[3] -dot(yAxis, cam.position);
                  zAxis[1] zAxis[2] zAxis[3] -dot(zAxis, cam.position);
                  0.0 0.0 0.0 1.0]
    return viewMatrix
end

function rotateAroundAxis(degrees::Float64, axis::Vector{Float32})
    ncos = 1 - cosd(degrees)
    sin = sind(degrees)
    cos = cosd(degrees)

    return [axis[1]^2*ncos+cos axis[1]*axis[2]*ncos-axis[3]*sin axis[1]*axis[3]*ncos+axis[2]*sin 0.0;
            axis[2]*axis[1]*ncos+axis[3]*sin axis[2]^2*ncos+cos axis[2]*axis[3]*ncos-axis[1]*sin 0.0;
            axis[1]*axis[3]*ncos-axis[2]*sin axis[2]*axis[3]*ncos+axis[1]*sin axis[3]^2*ncos+cos 0.0;
            0.0 0.0 0.0 1.0]
end

function checkCameraMovement(mousePos::Vector{Float64}, cam::Camera)
    if isRightMouseButtonDown
        difVector = mousePos - oldMousePosition

        camX = cam.speed * difVector[1]
        camY = cam.speed * difVector[2]

        rotateLeft(camX, cam)
        rotateUp(camY, cam)
    end 

    global oldMousePosition = mousePos
end

function rotateLeft(degrees::Float64, cam::Camera)
    pos = cam.position
	transformMatrix = rotateAroundAxis(degrees, cam.up);
	cam.position = deleteat!(transformMatrix * push!(pos, 1.0), 4);
end

function rotateUp(degrees::Float64, cam::Camera)
    sideAxis = cross(cam.up, cam.position);

    up = cam.up
    pos = cam.position
	transformMatrix = rotateAroundAxis(degrees, normalize!(sideAxis));
	cam.up = normalize!(deleteat!(transformMatrix * push!(up, 1.0), 4));
	cam.position = deleteat!(transformMatrix * push!(pos, 1.0), 4);
end