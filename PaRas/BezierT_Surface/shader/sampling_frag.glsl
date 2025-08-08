#version 430 core

in vec2 TexCoord;
in float Face;
in vec3 vNormal;

layout(location = 0) out vec4 fragColor;

void main()
{
    float u0 = TexCoord.x;
    float v0 = TexCoord.y;

    fragColor = vec4(u0, v0, 1.0f - u0 - v0, Face);
}