# version 330

layout(location=0) in vec3 position;
layout(location=1) in vec3 vertexColor;

uniform mat4 viewMatrix, projMatrix;

out vec3 color;

void main(void){
    color = vertexColor;
    gl_Position = projMatrix * viewMatrix * vec4(position, 1.0);
}