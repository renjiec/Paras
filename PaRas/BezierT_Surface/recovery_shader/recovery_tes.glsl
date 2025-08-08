#version 430

layout(triangles, equal_spacing, ccw) in;

in float faceID[];
out vec2 TexCoord;
out float Face;

uniform mat4 MVP;

vec3 compute_patch_center()
{
    vec3 center = vec3(0.0);
    for (int i = 0; i < 15; ++i)
    {
        center += gl_in[i].gl_Position.xyz;
    }
    center /= 15.0;
    return center;
}

void main()
{
    float u0 = gl_TessCoord.x;
    float v0 = gl_TessCoord.y;

    // Reassign
    vec4 b1 = gl_in[0].gl_Position;
    vec4 b2 = gl_in[1].gl_Position;
    vec4 b3 = gl_in[2].gl_Position;
    vec4 b4 = gl_in[3].gl_Position;
    vec4 b5 = gl_in[4].gl_Position;
    vec4 b6 = gl_in[5].gl_Position;
    vec4 b7 = gl_in[6].gl_Position;
    vec4 b8 = gl_in[7].gl_Position;
    vec4 b9 = gl_in[8].gl_Position;
    vec4 b10 = gl_in[9].gl_Position;
    vec4 b11 = gl_in[10].gl_Position;
    vec4 b12 = gl_in[11].gl_Position;
    vec4 b13 = gl_in[12].gl_Position;
    vec4 b14 = gl_in[13].gl_Position;
    vec4 b15 = gl_in[14].gl_Position;

    // Bezier interpolation
    vec4 TEPosition =
        u0 * u0 * u0 * u0 * b1 + 4 * u0 * u0 * u0 * v0 * b2 + 4 * u0 * u0 * u0 * (1 - u0 - v0) * b3 +
        6 * u0 * u0 * v0 * v0 * b4 + 12 * u0 * u0 * v0 * (1 - u0 - v0) * b5 + 6 * u0 * u0 * (1 - u0 - v0) * (1 - u0 - v0) * b6 +
        4 * u0 * v0 * v0 * v0 * b7 + 12 * u0 * v0 * v0 * (1 - u0 - v0) * b8 + 12 * u0 * v0 * (1 - u0 - v0) * (1 - u0 - v0) * b9 +
        4 * u0 * (1 - u0 - v0) * (1 - u0 - v0) * (1 - u0 - v0) * b10 + v0 * v0 * v0 * v0 * b11 + 4 * v0 * v0 * v0 * (1 - u0 - v0) * b12 +
        6 * v0 * v0 * (1 - u0 - v0) * (1 - u0 - v0) * b13 + 4 * v0 * (1 - u0 - v0) * (1 - u0 - v0) * (1 - u0 - v0) * b14 +
        (1 - u0 - v0) * (1 - u0 - v0) * (1 - u0 - v0) * (1 - u0 - v0) * b15;

    // Transform to clip coordinates
    float scale_amount = 1.2f;
    vec3 center = compute_patch_center();
    vec3 pos = TEPosition.xyz;
    vec3 dir = pos - center;
    pos = scale_amount * dir + center;
    gl_Position = MVP * vec4(pos, 1.0);

    // UV
    TexCoord = vec2(u0, v0);

    Face = faceID[0];
}
