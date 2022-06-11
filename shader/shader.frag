# version 330

in vec3 normal;
in vec3 worldPosition;

struct PointLight {
    vec3 position;
    vec3 color;
};

uniform vec3 ambientLightColor; 
uniform vec3 cameraPosition;   

// FIXED LIGHT: Maybe add more lights
int numLights = 1;
PointLight pointLight[1] = PointLight[1](PointLight(vec3(0.25, 0.5, 1.5), vec3(1.0, 1.0, 1.0)));

// FIXED MATERIAL:
vec3 color = vec3(1.0, 0.0, 0.0);
vec3 specularColor = vec3(1.0, 0.85, 0.85);
float ka = 0.3;
float kd = 0.5;
float ks = 0.8;
float shininess = 32.0;

out vec4 fragmentColor;

vec3 ComputeLambertian(const in vec3 dir_to_light, const in vec3 lightColor, const in vec3 normal, const in vec3 diffuseColor) {
        float nDotL = dot(normal, dir_to_light);         
        vec3 lambert = diffuseColor * lightColor * max (nDotL, 0.0); 
        return lambert;            
} 

vec3 ComputeSpecular(const in vec3 dir_to_light, const in vec3 dir_to_camera, const in vec3 lightColor, const in vec3 normal, const in vec3 specularColor, const in float shininess){
    vec3 r = 2 * dot(dir_to_light, normal) * normal - dir_to_light;

    float cosine = max(dot(dir_to_camera, r), 0.0); //if cosine is <0 then lock it to 0

    vec3 spec = lightColor * specularColor * pow(cosine, shininess); 
    return spec;
}

void main()                                                              
{
    vec3 ambient_color = ka*color;
    vec3 diffuse_color = kd*color;
    vec3 specular_color = ks*specularColor;

    vec3 myColor = ambientLightColor*ambient_color; 
    vec3 norm = normalize(normal);

    for (int i = 0; i < numLights; i++){
	   	vec3 dir_to_light = normalize(pointLight[i].position.xyz - myWorldPosition.xyz);
   		myColor += ComputeLambertian(dir_to_light, pointLight[i].color, norm, diffuse_color);

        vec3 dir_to_camera = normalize(cameraPosition - vec3(myWorldPosition));
        myColor += ComputeSpecular(dir_to_light, dir_to_camera, pointLight[i].color, norm, specular_color, material.shininess);
    }     

    fragmentColor = vec4(myColor, 1.0);
}