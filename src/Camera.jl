
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

"""
This method returns the rotational matrix in homogonous coordinates. 

# Arguments
`degrees::Float64`: The degrees to rotate.
`axis::Vector{Float32}`: The axis to rotate around. 

# Returns
`Matrix{Float64}`: 4x4 rotational matrix that describes the specified rotational behaviour.
"""
function rotateAroundAxis(degrees::Float64, axis::Vector{Float64})
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

        translationVector = vcat(-cam.position + [camX, camY, 0.0], 0.0)
        baseTransDist = norm(cam.position) # This should be 2
        sideAxis = cross(cam.up, cam.position)

        angleBetweenVecs(a, b) = acosd(clamp(a⋅b/(norm(a)*norm(b)), -1, 1))
        # angle between x axis and side axis and z axis and up vector
        θ = angleBetweenVecs([1.0, 0.0, 0.0], sideAxis)
        η = angleBetweenVecs([0.0, 1.0, 0.0], cam.up)

        transformMatrix = rotateAroundAxis(θ, [0.0, 1.0, 0.0]) * rotateAroundAxis(η, [1.0, 0.0, 0.0]) * hcat(Matrix(I,4,3), translationVector)

        # Modify camera position
        cam.up = normalize!(deleteat!(transformMatrix * push!(cam.up, 1.0), 4))
	    cam.position = deleteat!(transformMatrix * push!(cam.position, 1.0), 4);

        # undo matrix transformation
        invTransformMatrix = hcat(Matrix(I,4,3), baseTransDist*normalize!(translationVector)) * rotateAroundAxis(η, [1.0, 0.0, 0.0]) * rotateAroundAxis(θ, [0.0, 1.0, 0.0])

        cam.up = normalize!(deleteat!(invTransformMatrix * push!(cam.up, 1.0), 4))
	    cam.position = deleteat!(invTransformMatrix * push!(cam.position, 1.0), 4);

        # OLD CALCULATION
        # calculate side axis
        #sideAxis = cross(cam.up, cam.position);
        #transformMatrix = rotateAroundAxis(camX, cam.up) * rotateAroundAxis(camY, normalize!(sideAxis))

        #cam.up = normalize!(deleteat!(transformMatrix * push!(cam.up, 1.0), 4))
	    #cam.position = deleteat!(transformMatrix * push!(cam.position, 1.0), 4);
    end 
    
    #= 
    it could be tried to transform the camera coordinate in the global coordinate frame in the 
    origin. Then translate camera camX vertically and camY laterally. Remove the affects of the transform
    by mutiplying with the inverse of the transform matrix.
    Retrace steps and find wrong thoughts
    =#

    global oldMousePosition = mousePos
end
