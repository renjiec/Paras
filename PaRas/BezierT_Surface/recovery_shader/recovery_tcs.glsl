#version 430 core

layout(vertices = 15) out;

uniform float TessLevelInner;
uniform float TessLevelOuter;

layout(std430, binding = 0) buffer should_redraw_buffer {
    int should_redraw[];
};

in float _faceID[];
out float faceID[];

void main()
{
    // pass through
    gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
    faceID[gl_InvocationID] = _faceID[gl_InvocationID];

    if (gl_InvocationID == 0)
    {
        if (should_redraw[gl_PrimitiveID] == 0) {
            gl_TessLevelOuter[0] = 0;
            gl_TessLevelOuter[1] = 0;
            gl_TessLevelOuter[2] = 0;
            gl_TessLevelInner[0] = 0;
        }
        else {
            gl_TessLevelOuter[0] = 2;
            gl_TessLevelOuter[1] = 2;
            gl_TessLevelOuter[2] = 2;
            gl_TessLevelInner[0] = 2;
        }
    }
}
