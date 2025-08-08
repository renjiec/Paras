#version 430 core

layout(location = 0) in vec3 position;
layout(location = 1) in float showface;
layout(location = 2) in float faceID;

out float _faceID;

void main() {
    gl_Position = vec4(position, 1.0f);
    _faceID = faceID;
}