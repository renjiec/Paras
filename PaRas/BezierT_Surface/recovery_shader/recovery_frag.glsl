#version 430 core

in vec2 TexCoord;
in float Face;

layout(location = 0) out vec4 fragColor;
//layout(location = 1) out vec4 fragColor2;

void main()
{
    float u0 = TexCoord.x;
    float v0 = TexCoord.y;
    fragColor = vec4(u0, v0, 1.0f - u0 - v0, Face);
    //fragColor = vec4(1.0, 0.0, 1.0f, Face);
}