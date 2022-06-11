# version 330

layout(location=0) in vec3 position;
layout(location=1) in vec3 inNormal;

uniform mat4 viewMatrix, projMatrix, modelMatrix;

out vec3 worldPosition, normal;

void main(void){
    gl_Position = projMatrix * viewMatrix * modelMatrix * vec4(position, 1.0);

    worldPosition = modelMatrix * vec4(position, 1.0);
    normal = normalize(transpose(inverse(mat3(modelMatrix))) * inNormal)
}