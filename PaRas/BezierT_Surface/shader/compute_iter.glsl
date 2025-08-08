#version 430 core

layout(local_size_x = 10, local_size_y = 10, local_size_z = 1) in;

// Uniforms
layout(rgba32f, binding = 0) uniform image2D imgInput;
layout(std430, binding = 1) buffer world_position_buffer {
    vec4 pos[];
};
layout(std430, binding = 2) buffer coeff_buffer {
    vec2 coeff[];
};
layout(std430, binding = 3) buffer adj_face_buffer {
    int adjface[];
};
layout(std430, binding = 4) buffer should_redraw_buffer {
    int should_redraw[];
};

#define ep 0.01f
#define MAX_ITERATIONS 5
#define ERROR_THRESHOLD 0.1f

// 计算Bernstein基函数系数
vec2 computeBernsteinCoeff(int id, float u, float v, float w) {
    return u * u * u * u * coeff[id * 35 + 0] + u * u * u * v * coeff[id * 35 + 1] + u * u * u * w * coeff[id * 35 + 2] +
        u * u * v * v * coeff[id * 35 + 3] + u * u * v * w * coeff[id * 35 + 4] + u * u * w * w * coeff[id * 35 + 5] +
        u * v * v * v * coeff[id * 35 + 6] + u * v * v * w * coeff[id * 35 + 7] + u * v * w * w * coeff[id * 35 + 8] +
        u * w * w * w * coeff[id * 35 + 9] + v * v * v * v * coeff[id * 35 + 10] + v * v * v * w * coeff[id * 35 + 11] +
        v * v * w * w * coeff[id * 35 + 12] + v * w * w * w * coeff[id * 35 + 13] + w * w * w * w * coeff[id * 35 + 14];
}

// 计算导数系数
vec2 computeDerivativeCoeff(int id, float u, float v, float w, bool isU) {
    int offset = isU ? 15 : 25;
    return u * u * u * coeff[id * 35 + offset + 0] + u * u * w * coeff[id * 35 + offset + 1] +
        u * w * w * coeff[id * 35 + offset + 2] + w * w * w * coeff[id * 35 + offset + 3] +
        u * u * v * coeff[id * 35 + offset + 4] + u * w * v * coeff[id * 35 + offset + 5] +
        w * w * v * coeff[id * 35 + offset + 6] + u * v * v * coeff[id * 35 + offset + 7] +
        w * v * v * coeff[id * 35 + offset + 8] + v * v * v * coeff[id * 35 + offset + 9];
}

// 计算法线
vec3 computeNormal(int id, float u, float v, float w) {
    vec4 b[15];
    for (int i = 0; i < 15; i++) {
        b[i] = pos[15 * id + i];
    }

    vec4 du =
        4 * b[0] * u * u * u - 4 * b[2] * u * u * u + 12 * b[2] * u * u * w - 12 * b[5] * u * u * w -
        12 * b[9] * u * w * w + 12 * b[5] * u * w * w + 4 * b[9] * w * w * w - 4 * b[14] * w * w * w +
        12 * b[1] * u * u * v - 12 * b[4] * u * u * v + 24 * b[4] * u * w * v - 24 * b[8] * u * w * v -
        12 * b[13] * w * w * v + 12 * b[8] * w * w * v + 12 * b[3] * u * v * v - 12 * b[7] * u * v * v -
        12 * b[12] * w * v * v + 12 * b[7] * w * v * v - 4 * b[11] * v * v * v + 4 * b[6] * v * v * v;

    vec4 dv =
        4 * b[1] * u * u * u - 4 * b[2] * u * u * u + 12 * b[4] * u * u * w - 12 * b[5] * u * u * w -
        12 * b[9] * u * w * w + 12 * b[8] * u * w * w + 4 * b[13] * w * w * w - 4 * b[14] * w * w * w +
        12 * b[3] * u * u * v - 12 * b[4] * u * u * v + 24 * b[7] * u * w * v - 24 * b[8] * u * w * v +
        12 * b[12] * w * w * v - 12 * b[13] * w * w * v + 12 * b[6] * u * v * v - 12 * b[7] * u * v * v +
        12 * b[11] * w * v * v - 12 * b[12] * w * v * v + 4 * b[10] * v * v * v - 4 * b[11] * v * v * v;

    return normalize(cross(du.xyz, dv.xyz));
}

// 迭代法求解
bool Iteration(int id, float x0, float y0, inout float u, inout float v, inout float w, out vec3 normal_out) {
    float error = ERROR_THRESHOLD + 1.0f;
    int iter = 0;

    while (error > ERROR_THRESHOLD && iter < MAX_ITERATIONS) {
        iter++;

        vec2 Su = computeDerivativeCoeff(id, u, v, w, true);
        vec2 Sv = computeDerivativeCoeff(id, u, v, w, false);
        vec2 p = computeBernsteinCoeff(id, u, v, w);

        vec2 delta_p = vec2(x0, y0) - p;
        float det = Su.x * Sv.y - Sv.x * Su.y;
        if (abs(det) < 1e-6) break;

        float delta_u = dot(vec2(Sv.y, -Sv.x), delta_p) / det;
        float delta_v = dot(vec2(-Su.y, Su.x), delta_p) / det;

        u += delta_u;
        v += delta_v;
        w = 1.0f - u - v;

        p = computeBernsteinCoeff(id, u, v, w);
        error = length(p - vec2(x0, y0));
    }

    if (error <= ERROR_THRESHOLD &&
        u > -ep && u < 1.0f + ep &&
        v > -ep && v < 1.0f + ep &&
        u + v < 1.0f + ep) {

        //检查法向是否合法
        normal_out = computeNormal(id, u, v, w);
        float ndotv = dot(normal_out, vec3(0.0, 0.0, 1.0)); 
        if (ndotv < 0.0f) { 
            return false; 
        }
        return true;
    }
    return false;
}

void main() {
    ivec2 texCoord = ivec2(gl_GlobalInvocationID.xy);
    vec4 value = imageLoad(imgInput, texCoord);

    if (value.w < 1.01f) {
        return;
    }

    float x0 = 0.5f + texCoord.x;
    float y0 = 0.5f + texCoord.y;

    float u = value.x;
    float v = value.y;
    float w = 1.0f - u - v;
    int id = int(round(value.w - 2.0f));
	float ndotv;

    int adjIds[3] = { adjface[3 * id], adjface[3 * id + 1], adjface[3 * id + 2] };
    vec3 normal = computeNormal(id, u, v, w);

    // 尝试在当前面片求解
    if (Iteration(id, x0, y0, u, v, w, normal)) {
        imageStore(imgInput, texCoord, vec4(normal, id + 2.0f));

        ndotv = dot(normal, vec3(0.0, 0.0, 1.0));
        if (ndotv < 0.95f && should_redraw[id] == 0) {
            should_redraw[id] = 1;
        }
        return;
    }

    // 尝试在相邻面片求解
    for (int i = 0; i < 3; i++) {
        int adjId = adjIds[i] - 1;
        if (adjId < 0) continue;

        u = v = 0.33f;
        w = 1.0f - u - v;

        if (Iteration(adjId, x0, y0, u, v, w, normal)) {
            imageStore(imgInput, texCoord, vec4(normal, adjId + 2.0f));

            ndotv = dot(normal, vec3(0.0, 0.0, 1.0));
            if (ndotv < 0.95f && should_redraw[id] == 0) {
                should_redraw[id] = 1;
            }
            return;
        }
    }
	should_redraw[id] = 1;
}