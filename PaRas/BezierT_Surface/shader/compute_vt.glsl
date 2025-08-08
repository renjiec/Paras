#version 430 core

layout(local_size_x = 256, local_size_y = 1, local_size_z = 1) in;

layout(std430, binding = 0) buffer screen_position_buffer {
    vec2 screen_pos[];
};

layout(std430, binding = 1) buffer world_position_buffer {
    vec4 world_pos[];
};

layout(std430, binding = 2) buffer control_points_buffer {
    float pos[];
};

uniform float MVP[16];
uniform float width;
uniform float height;

void main() {

    uint index = gl_GlobalInvocationID.x; // 

    // 加载顶点位置到寄存器
    vec3 vertexPos = vec3(pos[3 * index], pos[3 * index + 1], pos[3 * index + 2]);

    // 预计算w分量
    float w =
        MVP[12] * vertexPos.x +
        MVP[13] * vertexPos.y +
        MVP[14] * vertexPos.z +
        MVP[15];

    // 计算透视除法前的坐标
    vec4 clipPos = vec4(
        MVP[0] * vertexPos.x + MVP[1] * vertexPos.y + MVP[2] * vertexPos.z + MVP[3],
        MVP[4] * vertexPos.x + MVP[5] * vertexPos.y + MVP[6] * vertexPos.z + MVP[7],
        MVP[8] * vertexPos.x + MVP[9] * vertexPos.y + MVP[10] * vertexPos.z + MVP[11],
        w
    );

    // 透视除法
    vec3 ndcPos = clipPos.xyz / w;

    // 存储结果
    world_pos[index] = vec4(ndcPos, 1.0);
    screen_pos[index] = vec2(
        (ndcPos.x + 1.0) * 0.5 * width,
        (ndcPos.y + 1.0) * 0.5 * height
    );
}

