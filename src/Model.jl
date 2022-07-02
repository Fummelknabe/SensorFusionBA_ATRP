# Used for model loading
using GLTF

mutable struct Transform
    position::Vector{Float32}
    eulerRotation::Vector{Float32}
    scale::Vector{Float32}

    function Transform()
        new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 1.0, 1.0])
    end
end

mutable struct Material
    diffuseColor::Vector{Float32}
    specularColor::Vector{Float32}
    ka::Float32
    kd::Float32
    ks::Float32
    shininess::Float32
end

mutable struct Mesh
    vao::UInt32
    ebo::UInt32
    sizeOfIndices::Int64
    transform::Transform
    material::Material
end

mutable struct Model
    meshes::Vector{Mesh}
    transform::Transform
end

function transformToMatrix(t::Transform)
    xRotation = GLfloat[1.0 0.0 0.0 0.0;
                        0.0 cos(t.eulerRotation[1]) -sin(t.eulerRotation[1]) 0.0;
                        0.0 sin(t.eulerRotation[1]) cos(t.eulerRotation[1]) 0.0;
                        0.0 0.0 0.0 1.0]
    yRotation = GLfloat[cos(t.eulerRotation[2]) 0.0 sin(t.eulerRotation[2]) 0.0;
                        0.0 1.0 0.0 0.0;
                        -sin(t.eulerRotation[2]) 0.0 cos(t.eulerRotation[2]) 0.0;
                        0.0 0.0 0.0 1.0]
    zRotation = GLfloat[cos(t.eulerRotation[3]) -sin(t.eulerRotation[3]) 0.0 0.0;
                        sin(t.eulerRotation[3]) cos(t.eulerRotation[3]) 0.0 0.0;
                        0.0 0.0 1.0 0.0;
                        0.0 0.0 0.0 1.0]

    scaleAndPos = GLfloat[t.scale[1] 0.0 0.0 t.position[1];
                          0.0 t.scale[2] 0.0 t.position[2];
                          0.0 0.0 t.scale[3] t.position[3];
                          0.0 0.0 0.0 1.0]

    return xRotation * yRotation * zRotation * scaleAndPos
end

function loadGLTFModelInBuffers(model::GLTF.Object, modelData::GLTF.ZVector)
    newModel = Model(Vector{Mesh}(undef, 0), Transform())
    accessors = model.accessors

    for mesh in model.meshes
        pos = accessors[mesh.primitives[0].attributes["POSITION"]]        
        texcoords = accessors[mesh.primitives[0].attributes["TEXCOORD_0"]]        
        normals = accessors[mesh.primitives[0].attributes["NORMAL"]]        
        indices = accessors[mesh.primitives[0].indices]   
        material = model.materials[mesh.primitives[0].material]   

        mesh = loadGLTFMeshInBuffers(pos, texcoords, normals, indices, material, model, modelData)
        push!(newModel.meshes, mesh)
    end

    return newModel
end

function loadGLTFMeshInBuffers(pos::GLTF.Accessor, texcoords::GLTF.Accessor, normals::GLTF.Accessor, indices::GLTF.Accessor, material, model::GLTF.Object, modelData::GLTF.ZVector)
    # Create Buffer views
    posBufferView = model.bufferViews[pos.bufferView]
    texBufferView = model.bufferViews[texcoords.bufferView]
    normalBufferView = model.bufferViews[normals.bufferView]
    idxBufferView = model.bufferViews[indices.bufferView]

    # create buffers located in the memory of graphic card
    # position
    posVBO = GLuint(0)
    @c glGenBuffers(1, &posVBO)
    glBindBuffer(posBufferView.target, posVBO)
    glBufferData(posBufferView.target, posBufferView.byteLength, C_NULL, GL_STATIC_DRAW)
    posData = modelData[posBufferView.buffer]
    @c glBufferSubData(posBufferView.target, 0, posBufferView.byteLength, &posData[posBufferView.byteOffset])
    # normals
    normalsVBO = GLuint(0)
    @c glGenBuffers(1, &normalsVBO)
    glBindBuffer(normalBufferView.target, normalsVBO)
    glBufferData(normalBufferView.target, normalBufferView.byteLength, C_NULL, GL_STATIC_DRAW)
    normalData = modelData[normalBufferView.buffer]
    @c glBufferSubData(normalBufferView.target, 0, normalBufferView.byteLength, &normalData[normalBufferView.byteOffset])
    # texure coordinates
    texVBO = GLuint(0)
    @c glGenBuffers(1, &texVBO)
    glBindBuffer(texBufferView.target, texVBO)
    glBufferData(texBufferView.target, texBufferView.byteLength, C_NULL, GL_STATIC_DRAW)
    texData = modelData[texBufferView.buffer]
    @c glBufferSubData(texBufferView.target, 0, texBufferView.byteLength, &texData[texBufferView.byteOffset])
    # indices
    idxEBO = GLuint(0)
    @c glGenBuffers(1, &idxEBO)
    glBindBuffer(idxBufferView.target, idxEBO)
    glBufferData(idxBufferView.target, idxBufferView.byteLength, C_NULL, GL_STATIC_DRAW)
    idxData = modelData[idxBufferView.buffer]
    @c glBufferSubData(idxBufferView.target, 0, idxBufferView.byteLength, &idxData[idxBufferView.byteOffset])

    # create VAO
    vao = GLuint(0)
    @c glGenVertexArrays(1, &vao)
    glBindVertexArray(vao)
    glBindBuffer(posBufferView.target, posVBO)
    glVertexAttribPointer(0, 3, pos.componentType, pos.normalized, posBufferView.byteStride, Ptr{Cvoid}(pos.byteOffset))
    glBindBuffer(normalBufferView.target, normalsVBO)
    glVertexAttribPointer(1, 3, normals.componentType, normals.normalized, normalBufferView.byteStride, Ptr{Cvoid}(normals.byteOffset))
    glBindBuffer(idxBufferView.target, texVBO)
    glVertexAttribPointer(2, 2, texcoords.componentType, texcoords.normalized, texBufferView.byteStride, Ptr{Cvoid}(texcoords.byteOffset))
    glEnableVertexAttribArray(0)
    glEnableVertexAttribArray(1)
    glEnableVertexAttribArray(2)

    # unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0)
	glBindVertexArray(0)

    # Extract material values
    newMaterial = Material(
        material.pbrMetallicRoughness.baseColorFactor[1:3],
        material.pbrMetallicRoughness.baseColorFactor[1:3],
        material.emissiveFactor[1],
        material.emissiveFactor[2],
        material.emissiveFactor[3],
        material.pbrMetallicRoughness.metallicFactor
    )

    return Mesh(vao, idxEBO, indices.count, Transform(), newMaterial)
end

function writeToUniforms(program, transformMatrix::Matrix, cam::Camera, ambientLightColor::Vector{GLfloat}, material::Material)
    modelMatrixLoc = glGetUniformLocation(program, "modelMatrix")
    viewMatrixLoc = glGetUniformLocation(program, "viewMatrix")
    projMatrixLoc = glGetUniformLocation(program, "projMatrix")
    camPositionLoc = glGetUniformLocation(program, "cameraPosition")
    ambientLightCoLoc = glGetUniformLocation(program, "ambientLightColor")
    glUseProgram(program)
    glUniformMatrix4fv(modelMatrixLoc, 1, GL_FALSE, transformMatrix)
    glUniformMatrix4fv(viewMatrixLoc, 1, GL_FALSE, getViewMatrix(cam))
    glUniformMatrix4fv(projMatrixLoc, 1, GL_FALSE, getProjectionMatrix(cam))
    glUniform3fv(camPositionLoc, 1, cam.position)
    glUniform3fv(ambientLightCoLoc, 1, ambientLightColor)
    glUniform3fv(glGetUniformLocation(program, "material.diffuseColor"), 1, material.diffuseColor)
    glUniform3fv(glGetUniformLocation(program, "material.specularColor"), 1, material.specularColor)
    glUniform1f(glGetUniformLocation(program, "material.ka"), material.ka)
    glUniform1f(glGetUniformLocation(program, "material.kd"), material.kd)
    glUniform1f(glGetUniformLocation(program, "material.ks"), material.ks)
    glUniform1f(glGetUniformLocation(program, "material.shininess"), material.shininess)
end