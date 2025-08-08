#version 430

layout(triangles, equal_spacing, ccw) in;

out vec2 TexCoord;
out float Face;
out vec3 vNormal;

in float faceID[];

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
//uniform mat4 MV;

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

    // Compute the normal vector
    vec4 du =
        4 * b1 * u0 * u0 * u0 - 4 * b3 * u0 * u0 * u0 + 12 * b3 * u0 * u0 * (1 - u0 - v0) - 12 * b6 * u0 * u0 * (1 - u0 - v0) -
        12 * b10 * u0 * (1 - u0 - v0) * (1 - u0 - v0) + 12 * b6 * u0 * (1 - u0 - v0) * (1 - u0 - v0) +
        4 * b10 * (1 - u0 - v0) * (1 - u0 - v0) * (1 - u0 - v0) - 4 * b15 * (1 - u0 - v0) * (1 - u0 - v0) * (1 - u0 - v0) + 12 * b2 * u0 * u0 * v0 -
        12 * b5 * u0 * u0 * v0 + 24 * b5 * u0 * (1 - u0 - v0) * v0 - 24 * b9 * u0 * (1 - u0 - v0) * v0 -
        12 * b14 * (1 - u0 - v0) * (1 - u0 - v0) * v0 + 12 * b9 * (1 - u0 - v0) * (1 - u0 - v0) * v0 + 12 * b4 * u0 * v0 * v0 -
        12 * b8 * u0 * v0 * v0 - 12 * b13 * (1 - u0 - v0) * v0 * v0 + 12 * b8 * (1 - u0 - v0) * v0 * v0 -
        4 * b12 * v0 * v0 * v0 + 4 * b7 * v0 * v0 * v0;

    vec4 dv =
        4 * b2 * u0 * u0 * u0 - 4 * b3 * u0 * u0 * u0 + 12 * b5 * u0 * u0 * (1 - u0 - v0) - 12 * b6 * u0 * u0 * (1 - u0 - v0) -
        12 * b10 * u0 * (1 - u0 - v0) * (1 - u0 - v0) + 12 * b9 * u0 * (1 - u0 - v0) * (1 - u0 - v0) +
        4 * b14 * (1 - u0 - v0) * (1 - u0 - v0) * (1 - u0 - v0) - 4 * b15 * (1 - u0 - v0) * (1 - u0 - v0) * (1 - u0 - v0) + 12 * b4 * u0 * u0 * v0 -
        12 * b5 * u0 * u0 * v0 + 24 * b8 * u0 * (1 - u0 - v0) * v0 - 24 * b9 * u0 * (1 - u0 - v0) * v0 +
        12 * b13 * (1 - u0 - v0) * (1 - u0 - v0) * v0 - 12 * b14 * (1 - u0 - v0) * (1 - u0 - v0) * v0 + 12 * b7 * u0 * v0 * v0 -
        12 * b8 * u0 * v0 * v0 + 12 * b12 * (1 - u0 - v0) * v0 * v0 - 12 * b13 * (1 - u0 - v0) * v0 * v0 +
        4 * b11 * v0 * v0 * v0 - 4 * b12 * v0 * v0 * v0;

    //Normal = normalize(cross(du.xyz, dv.xyz)));
    vec3 dPdu = vec3(du);
    vec3 dPdv = vec3(dv);
    vec3 normal_model = normalize(cross(dPdu, dPdv));

    mat3 normalMatrix = transpose(inverse(mat3(view * model)));
    vNormal = normalize(normalMatrix * normal_model);

    // Transform to clip coordinates
    float scale_amount = 1.f;
    vec3 center = compute_patch_center();
    vec3 pos = TEPosition.xyz;
    vec3 dir = pos - center;
    pos = scale_amount * dir + center; // 向外放大
    gl_Position = projection * view * model * vec4(pos, 1.0);


    //gl_Position = projection * view * model * TEPosition;

    // UV
    TexCoord = vec2(u0, v0);

    Face = faceID[0];
}
