#version 450

in vec2 TexCoord;

layout(location = 0) out vec4 FragColor;
uniform sampler2D gNormalAndDepth;
uniform bool show_patch;

const vec3 lightDir = normalize(vec3(1.0, 1.0, 1.0));
const vec3 viewDir  = normalize(vec3(0.0, 0.0, 1.0));
const vec3 lightColor = vec3(1.0);
const vec3 objectColor = vec3(0.8, 0.4, 0.3);

void main() {
    float z = texture(gNormalAndDepth, TexCoord).a;

    if (abs(z - 1.0f) <= 0.01f) {
    FragColor = vec4(1.0f, 1.0f, 1.0f, 1.0f);
    }
    else {
    
   
    float ss = floor(texture(gNormalAndDepth, TexCoord).a + 0.01f);
        float k = float(floor(ss / 10.0f));
        float randomValue = abs(10.0f * k - ss);
        float r2, g2, b2;

        if (randomValue <= 0.5f) {
            r2 = 145.0f / 255.0f;
            g2 = 204.0f / 255.0f;
            b2 = 192.0f / 255.0f;
        }
        else if (randomValue <= 1.0f) {
            r2 = 127.0f / 255.0f;
            g2 = 171.0f / 255.0f;
            b2 = 209.0f / 255.0f;
        }
        else if (randomValue <= 2.0f) {
            r2 = 238.0f / 255.0f;
            g2 = 182.0f / 255.0f;
            b2 = 212.0f / 255.0f;
        }
        else if (randomValue <= 3.0f) {
            r2 = 236.0f / 255.0f;
            g2 = 110.0f / 255.0f;
            b2 = 102.0f / 255.0f;
        }
        else if (randomValue <= 4.0f) {
            r2 = 181.0f / 255.0f;
            g2 = 206.0f / 255.0f;
            b2 = 78.0f / 255.0f;
        }
        else if (randomValue <= 5.0f) {
            r2 = 245.0f / 255.0f;
            g2 = 235.0f / 255.0f;
            b2 = 174.0f / 255.0f;
        }
        else if (randomValue <= 6.0f) {
            r2 = 151.0f / 255.0f;
            g2 = 208.0f / 255.0f;
            b2 = 197.0f / 255.0f;
        }
        else if (randomValue <= 7.0f) {
            r2 = 190.0f / 255.0f;
            g2 = 208.0f / 255.0f;
            b2 = 249.0f / 255.0f;
        }
        else if (randomValue <= 8.0f) {
            r2 = 82.0f / 255.0f;
            g2 = 170.0f / 255.0f;
            b2 = 220.0f / 255.0f;
        }
        else if (randomValue <= 9.0f) {
            r2 = 199.0f / 255.0f;
            g2 = 103.0f / 255.0f;
            b2 = 222.0f / 255.0f;
        }
        else {
            r2 = 225.0f / 255.0f;
            g2 = 225.0f / 255.0f;
            b2 = 225.0f / 255.0f;
        }
        // Assign colors based on the random value

    // result
    vec3 uvColor = texture(gNormalAndDepth, TexCoord).xyz;
    vec3 patchColor = vec3(r2, g2, b2);

    vec3 normal = texture(gNormalAndDepth, TexCoord).xyz;
    vec3 halfwayDir = normalize(lightDir + viewDir);

    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * lightColor;

    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    float specStrength = 0.5;
    float shininess = 64.0;
    float spec = pow(max(dot(normal, halfwayDir), 0.0), shininess);
    vec3 specular = specStrength * spec * lightColor;

    vec3 color = show_patch ? patchColor : objectColor;
    vec3 result = (ambient + diffuse + specular) * color;
    FragColor = vec4(result, 1.0);

    }
}