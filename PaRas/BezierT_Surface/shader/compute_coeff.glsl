#version 430 core

layout(local_size_x = 256, local_size_y = 1, local_size_z = 1) in;

layout(std430, binding = 0) buffer coeff_buffer {
    vec2 coeff[];
};

layout(std430, binding = 1) buffer screen_position_buffer {
    vec2 screen_pos[];
};

void main() {
    uint index = gl_GlobalInvocationID.x; //

    // 预加载所有需要的屏幕位置到局部变量
    vec2 sp[15];
    for (int i = 0; i < 15; ++i) {
        sp[i] = screen_pos[15 * index + i];
    }

    // 第一部分：直接缩放计算
    coeff[35 * index + 0] = 1.0f * sp[0];
    coeff[35 * index + 1] = 4.0f * sp[1];
    coeff[35 * index + 2] = 4.0f * sp[2];
    coeff[35 * index + 3] = 6.0f * sp[3];
    coeff[35 * index + 4] = 12.0f * sp[4];
    coeff[35 * index + 5] = 6.0f * sp[5];
    coeff[35 * index + 6] = 4.0f * sp[6];
    coeff[35 * index + 7] = 12.0f * sp[7];
    coeff[35 * index + 8] = 12.0f * sp[8];
    coeff[35 * index + 9] = 4.0f * sp[9];
    coeff[35 * index + 10] = 1.0f * sp[10];
    coeff[35 * index + 11] = 4.0f * sp[11];
    coeff[35 * index + 12] = 6.0f * sp[12];
    coeff[35 * index + 13] = 4.0f * sp[13];
    coeff[35 * index + 14] = 1.0f * sp[14];

    // 第二部分：差值计算
    coeff[35 * index + 15] = 4.0f * (sp[0] - sp[2]);
    coeff[35 * index + 16] = 12.0f * (sp[2] - sp[5]);
    coeff[35 * index + 17] = 12.0f * (sp[5] - sp[9]);
    coeff[35 * index + 18] = 4.0f * (sp[9] - sp[14]);
    coeff[35 * index + 19] = 12.0f * (sp[1] - sp[4]);
    coeff[35 * index + 20] = 24.0f * (sp[4] - sp[8]);
    coeff[35 * index + 21] = 12.0f * (sp[8] - sp[13]);
    coeff[35 * index + 22] = 12.0f * (sp[3] - sp[7]);
    coeff[35 * index + 23] = 12.0f * (sp[7] - sp[12]);
    coeff[35 * index + 24] = 4.0f * (sp[6] - sp[11]);

    // 第三部分：差值计算
    coeff[35 * index + 25] = 4.0f * (sp[1] - sp[2]);
    coeff[35 * index + 26] = 12.0f * (sp[4] - sp[5]);
    coeff[35 * index + 27] = 12.0f * (sp[8] - sp[9]);
    coeff[35 * index + 28] = 4.0f * (sp[13] - sp[14]);
    coeff[35 * index + 29] = 12.0f * (sp[3] - sp[4]);
    coeff[35 * index + 30] = 24.0f * (sp[7] - sp[8]);
    coeff[35 * index + 31] = 12.0f * (sp[12] - sp[13]);
    coeff[35 * index + 32] = 12.0f * (sp[6] - sp[7]);
    coeff[35 * index + 33] = 12.0f * (sp[11] - sp[12]);
    coeff[35 * index + 34] = 4.0f * (sp[10] - sp[11]);

}