"""
This function transforms the camera position so that the z-vector points up in the intertial frame 

# Arguments
`camPos::Vector{Float32}`: The camera position to transform
`up::Vector{Float32}`: The vector that holds the acceleration data 

# Returns 
`Vector{Float32}`: The transformed position
"""
function transformCamCoords(camPos::Vector{Float32}, up::Vector{Float32})
    # Projection in planes
    projXZ = [up[1], 0.0, -up[3]]
    projYZ = [0.0, up[2], -up[3]]

    # Angle between axis
    α = -acos(projXZ[3] / norm(projXZ)) 
    β = -acos(projYZ[3] / norm(projYZ))

    Rx = [1     0      0;
          0 cos(β) -sin(β); 
          0 sin(β) cos(β)]    
    Ry = [cos(α) 0 sin(α);
          0      1      0;
          -sin(α) 0 cos(α)]

    return (Rx*Ry)*camPos
end