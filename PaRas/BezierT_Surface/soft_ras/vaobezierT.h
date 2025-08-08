#pragma once
#include <algorithm>
#include <vector>
#include <array>
#include <chrono>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include "glprogram.h"
#include "glarray.h"

//#include <cuda_runtime.h>
//#include <cuda_gl_interop.h>
//#include "soft_ras/soft_rasterize_cuda.cuh"

Eigen::Matrix3f quaternaion2matrix(const float* q)
{
    double s = Eigen::Map<const Eigen::Vector4f>(q).squaredNorm();
    Eigen::Matrix3f res;

    float* m = res.data();

    m[0] = 1.f - 2.f * (q[1] * q[1] + q[2] * q[2]);
    m[1] = 2.f * (q[0] * q[1] - q[2] * q[3]);
    m[2] = 2.f * (q[2] * q[0] + q[1] * q[3]);

    m[3 + 0] = 2.f * (q[0] * q[1] + q[2] * q[3]);
    m[3 + 1] = 1.f - 2.f * (q[2] * q[2] + q[0] * q[0]);
    m[3 + 2] = 2.f * (q[1] * q[2] - q[0] * q[3]);

    m[6 + 0] = 2.f * (q[2] * q[0] - q[1] * q[3]);
    m[6 + 1] = 2.f * (q[1] * q[2] + q[0] * q[3]);
    m[6 + 2] = 1.f - 2.f * (q[1] * q[1] + q[0] * q[0]);

    return res.transpose();
}

Eigen::Matrix4f perspective(float fovy, float aspect, float zNear, float zFar)
{
    assert(aspect > 0);
    assert(zFar > zNear);

    float radf = fovy / 180 * 3.1415926f;
    float tanHalfFovy = tan(radf / 2);
    Eigen::Matrix4f res = Eigen::Matrix4f::Zero();
    res(0, 0) = 1 / (aspect * tanHalfFovy);
    res(1, 1) = 1 / (tanHalfFovy);
    res(2, 2) = -(zFar + zNear) / (zFar - zNear);
    res(3, 2) = -1;
    res(2, 3) = -(2 * zFar * zNear) / (zFar - zNear);
    return res;
}


Eigen::Matrix4f orthogonal(float l, float r, float t, float b, float n, float f)
{
    Eigen::Matrix4f res = Eigen::Matrix4f::Zero();
    res(0, 0) = 2 / (r - l);
    res(1, 1) = 2 / (t - b);
    res(2, 2) = 2 / (n - f);
    res(0, 2) = -(r + l) / (r - l);
    res(1, 2) = -(t + b) / (t - b);
    res(2, 2) = -(n + f) / (n - f);
    res(3, 3) = 1;
    return res;
}

Eigen::Matrix4f set_view(float z)
{
    Eigen::Matrix4f res = Eigen::Matrix4f::Zero();
    res(0, 0) = 1.0f;
    res(1, 1) = 1.0f;
    res(2, 2) = 1.0f;
    res(3, 3) = 1.0f;
    res(2, 3) = -z;

    return res;
}

Eigen::Matrix4f set_model()
{
    Eigen::Matrix4f res = Eigen::Matrix4f::Zero();
    res(0, 0) = 1.5f;
    res(1, 1) = 1.5f;
    res(2, 2) = 1.5f;
    res(3, 3) = 1.0f;

    return res;
}


Eigen::Matrix4f lookAt(const float* eye, const float* center, const float* up)
{
    using Vec = Eigen::RowVector3f;
    using MapVec = Eigen::Map<const Vec>;
    Vec f = (MapVec(center) - MapVec(eye)).normalized();
    Vec u = MapVec(up).normalized();
    Vec s = f.cross(u).normalized();
    u = s.cross(f);

    Eigen::Matrix4f res;
    res.leftCols(3) << s, u, -f, 0, 0, 0;
    res.rightCols(1) << -res.topLeftCorner(3, 3) * MapVec(eye).transpose(), 1;
    return res;
}

class GLFbo
{
public:
    GLuint gNormalAndDepth, rboDepth, gBuffer, compute_texture;
    GLFbo() :gNormalAndDepth(0), rboDepth(0), gBuffer(0), compute_texture(0) {};
    ~GLFbo() { deleteFbo(); };
    void createFbo(int WIDTH, int HEIGHT)
    {
        glGenFramebuffers(1, &gBuffer);
        glBindFramebuffer(GL_FRAMEBUFFER, gBuffer);

        glGenTextures(1, &gNormalAndDepth);
        glBindTexture(GL_TEXTURE_2D, gNormalAndDepth);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, WIDTH, HEIGHT, 0, GL_RGBA, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, gNormalAndDepth, 0);

        glGenTextures(1, &compute_texture);
        glBindTexture(GL_TEXTURE_2D, compute_texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, WIDTH, HEIGHT, 0, GL_RGBA, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, compute_texture, 0);

        unsigned int attachments[2] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
        glDrawBuffers(2, attachments);

        //unsigned int rboDepth;
        glGenRenderbuffers(1, &rboDepth);
        glBindRenderbuffer(GL_RENDERBUFFER, rboDepth);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, WIDTH, HEIGHT);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rboDepth);
        // check if framebuffer is complete
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            std::cout << "Framebuffer not complete!" << std::endl;
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    void deleteFbo()
    {
        glDeleteTextures(1, &gNormalAndDepth);
        glDeleteTextures(1, &compute_texture);
        glDeleteRenderbuffers(1, &rboDepth);
        glDeleteFramebuffers(1, &gBuffer);
    }

    void bind() { glBindFramebuffer(GL_FRAMEBUFFER, gBuffer); };
    void unbind() { glBindFramebuffer(GL_FRAMEBUFFER, 0); };
};

template<typename R = float, int vertex_dimension = 3>
struct GLBezierT
{
    enum { dim = vertex_dimension};

    typedef std::array<R, 4> vec4;
    typedef std::array<R, 3> vec3;
    typedef std::array<R, 2> vec2;

    struct ControlPoints
    {
        std::vector<vec3> vertices;
        //std::vector<vec2> texCoord;
        std::vector<std::array<int, 3>> adjfaces;
        std::vector<float> faceID;
        std::vector<float> showface;
        size_t nVertex() const { return vertices.size(); }
        size_t nFace() const { return vertices.size() / (15); }
    };
    ControlPoints control_points;

    GLFbo fbo;
    GLFbo samplingFBO;

    unsigned int texture;

    //Rasterizer rasterizer; //gpu data

    int tess_level_inner = 1;
    int tess_level_outer = 1;

    float mMeshScale = 1.f;
    vec3 mTranslate = { 0, 0, 0 };
    float mQRotate[4] = { 0, 0, 0, 1 };
    float mViewCenter[3] = { 0, 0, 0 };
    R bbox[dim * 2];

    static GLProgram prog_recovery;
    static GLProgram prog_soft_ras;
    static GLProgram prog_screen;

    //compute shader prog
    static GLProgram prog_compute;
    static GLProgram prog_compute_coeff;
    static GLProgram prog_compute_iter; 
    static GLProgram prog_compute_recovery;

    int nVertex; // the number of control points
    int nFace;

    GLuint vaoHandle;

    // cuda interp
    //cudaGraphicsResource* cudaResource;
    //cudaGraphicsResource* cudaResource2;
    //cudaArray* cuda_image_array;
    //cudaArray* cuda_tex_array;
    //float1* dptr;
    //size_t num_bytes;

    /*quad in screen*/
    GLuint quadVAO = 0;
    GLuint quadVBO;

    GLuint bezierVAO = 0;
    GLuint bezierVBO;
    GLuint bezierVBO2;
    GLuint bezierVBO3;

    // compute shader buffers
    GLuint control_points_buffer;
    GLuint world_position_buffer;
    GLuint screen_position_buffer;
    GLuint coeff_buffer;
    GLuint adj_face_buffer;
    GLuint should_redraw_buffer;

    // light
    vec4 lightColor = { 1.0,1.0,1.0,1.0 };
    vec3 lightPos = {  -3.0f, -2.0f, -3.0f };
    vec4 faceColor = { 1.f, 1.f, 1.f, 1.f };

    float light_ambient = 0.2f;
    float light_diffuse = 0.7f;
    float light_specular = 1.0f;

    // material
    float material_shiness = 64.0f;
    float material_ambient = 1.0f;
    float material_diffuse = 0.9f;
    float material_specular = 0.5f;
    
    // rendering mode
    //bool show_error = false;
    //bool iter = true;
    //bool show_normal = false;
    bool show_contour = true;
    bool show_patch = true;

    // timing
    float fps = 0.0f;

    float campos[3] = { 0, 0, 5.5 }, up[3] = { 0, 1, 0 };

    ~GLBezierT() 
    { 
        glDeleteVertexArrays(1, &vaoHandle); 
    }

    GLBezierT() :vaoHandle(0)
    {
        ;
    }

    bool loadFromFile(const std::string& vertexFile, const std::string& connFile)
    {
        using std::cout;
        using std::endl;

        // ---------- Load control points ----------
        std::ifstream file(vertexFile);
        if (!file.is_open()) {
            cout << "Failed to open vertex file: " << vertexFile << endl;
            return false;
        }

        float v1, v2, v3;
        while (file >> v1 >> v2 >> v3) {
            control_points.vertices.push_back({ v1, v2, v3 });
        }
        file.close();

        size_t nFaces = control_points.nFace();

        control_points.faceID.reserve(nFaces * 15);
        control_points.showface.reserve(nFaces * 15);

        for (size_t i = 0; i < nFaces; ++i) {
            for (int j = 0; j < 15; ++j) {
                control_points.faceID.push_back(static_cast<float>(i) + 2.0f);
                control_points.showface.push_back(0.0f);
            }
        }

        // ---------- Load adjacency info ----------
        std::ifstream conn(connFile);
        if (!conn.is_open()) {
            cout << "Failed to open connection file: " << connFile << endl;
            return false;
        }

        int a, b, c;
        while (conn >> a >> b >> c) {
            control_points.adjfaces.push_back({ a, b, c });
        }
        conn.close();

        // Set metadata
        nVertex = static_cast<int>(control_points.vertices.size());
        nFace = static_cast<int>(control_points.nFace());

        cout << "Loaded " << nFace << " faces and " << nVertex << " control points." << endl;
        return true;
    }

    bool loadFromBinaryFile(const std::string& binaryFile) {
        using std::cout;
        using std::endl;

        std::ifstream file(binaryFile, std::ios::binary);
        if (!file.is_open()) {
            cout << "Failed to open binary file: " << binaryFile << endl;
            return false;
        }

        // 读取顶点数
        int num_vertices = 0;
        file.read(reinterpret_cast<char*>(&num_vertices), sizeof(int));

        control_points.vertices.clear();
        control_points.vertices.reserve(num_vertices);

        for (int i = 0; i < num_vertices; ++i) {
            float v[3];
            file.read(reinterpret_cast<char*>(v), sizeof(float) * 3);
            control_points.vertices.push_back({ v[0], v[1], v[2] });
        }

        // 读取邻接数
        int num_adj = 0;
        file.read(reinterpret_cast<char*>(&num_adj), sizeof(int));

        control_points.adjfaces.clear();
        control_points.adjfaces.reserve(num_adj);

        for (int i = 0; i < num_adj; ++i) {
            int a, b, c;
            file.read(reinterpret_cast<char*>(&a), sizeof(int));
            file.read(reinterpret_cast<char*>(&b), sizeof(int));
            file.read(reinterpret_cast<char*>(&c), sizeof(int));
            control_points.adjfaces.push_back({ a, b, c });
        }

        file.close();

        // 设置面数和顶点数
        nVertex = num_vertices;
        nFace = static_cast<int>(control_points.nFace());

        control_points.faceID.reserve(nFace * 15);
        control_points.showface.reserve(nFace * 15);
        for (int i = 0; i < nFace; ++i) {
            for (int j = 0; j < 15; ++j) {
                control_points.faceID.push_back(static_cast<float>(i) + 2.0f);
                control_points.showface.push_back(0.0f);
            }
        }
        cout << "Loaded " << nFace << " faces and " << nVertex << " control points." << endl;
        return true;
    }

    R drawscale() const
    {
        R scale0 = 1.9f / std::max(std::max(bbox[dim] - bbox[0], bbox[1 + dim] - bbox[1]), bbox[2 + dim] - bbox[2]);
        return mMeshScale * scale0;
    }

    //void setMVP(const int* vp, bool offsetDepth = false, bool colmajor = false) {
    //    R trans[] = { (bbox[0] + bbox[dim]) / 2, (bbox[1] + bbox[1 + dim]) / 2, (bbox[2] + bbox[2 + dim]) / 2 };
    //    R ss = drawscale();
    //    R t[] = { mTranslate[0] - trans[0], mTranslate[1] - trans[1], mTranslate[2] - trans[2] };

    //    using Mat4 = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;

    //    Mat4 proj = perspective(35.f, vp[2] / float(vp[3]), 1.5f, 100.0f);
    //    //Mat4 proj = perspective(45.f, 1280.0f / 720.0f, 1.5f, 100.0);

    //    float up[] = { 0, 1, 0 };// Head is up (set to 0,-1,0 to look upside-down)
    //    Mat4 view = lookAt(campos, mViewCenter, up);
    //    //Mat4 view = set_view(campos[2]);

    //    Mat4 model;
    //    model << ss * quaternaion2matrix(mQRotate), Eigen::Map<Eigen::Array3f>(t)* ss,
    //        0, 0, 0, 1;
    //    //model = set_model();

    //    Mat4 mvp;
    //    mvp = proj * view * model;
    //    rasterizer.gpu_vertices.set_matrix(model.data(), view.data(), proj.data(), mvp.data());
    //}



    std::array<R, 16> matMVP(const int* vp, bool offsetDepth = false, bool colmajor = false) const {
        R trans[] = { (bbox[0] + bbox[dim]) / 2, (bbox[1] + bbox[1 + dim]) / 2, (bbox[2] + bbox[2 + dim]) / 2 };
        R ss = drawscale();
        R t[] = { mTranslate[0] - trans[0], mTranslate[1] - trans[1], mTranslate[2] - trans[2] };

        using Mat4 = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;

        Mat4 proj = perspective(35.f, vp[2] / float(vp[3]), 1.5f, 100.0f);
        //Mat4 proj = perspective(45.f, 1280.0f / 720.0f, 1.5f, 100.0);

        float up[] = { 0, 1, 0 };// Head is up (set to 0,-1,0 to look upside-down)
        Mat4 view = lookAt(campos, mViewCenter, up);
        //Mat4 view = set_view(campos[2]);

        Mat4 model;
        model << ss * quaternaion2matrix(mQRotate), Eigen::Map<Eigen::Array3f>(t)* ss,
            0, 0, 0, 1;
        //model = set_model();

        std::array<R, 16> mvp;
        Eigen::Map<Mat4>(mvp.data()) = proj * view * model;
        return mvp;
    }

    std::array<R, 16> matMV(const int* vp, bool offsetDepth = false, bool colmajor = false) const {
        R trans[] = { (bbox[0] + bbox[dim]) / 2, (bbox[1] + bbox[1 + dim]) / 2, (bbox[2] + bbox[2 + dim]) / 2 };
        R ss = drawscale();
        R t[] = { mTranslate[0] - trans[0], mTranslate[1] - trans[1], mTranslate[2] - trans[2] };

        using Mat4 = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;

        float up[] = { 0, 1, 0 };// Head is up (set to 0,-1,0 to look upside-down)
        Mat4 view = lookAt(campos, mViewCenter, up);

        Mat4 model;
        model << ss * quaternaion2matrix(mQRotate), Eigen::Map<Eigen::Array3f>(t)* ss,
            0, 0, 0, 1;

        std::array<R, 16> mvp;
        Eigen::Map<Mat4>(mvp.data()) = view * model;
        return mvp;
    }

    std::array<R, 16> matM(const int* vp, bool offsetDepth = false, bool colmajor = false) const {
        R trans[] = { (bbox[0] + bbox[dim]) / 2, (bbox[1] + bbox[1 + dim]) / 2, (bbox[2] + bbox[2 + dim]) / 2 };
        R ss = drawscale();
        R t[] = { mTranslate[0] - trans[0], mTranslate[1] - trans[1], mTranslate[2] - trans[2] };

        using Mat4 = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;

        Mat4 model;
        model << ss * quaternaion2matrix(mQRotate), Eigen::Map<Eigen::Array3f>(t)* ss,
            0, 0, 0, 1;

        std::array<R, 16> mvp;
        Eigen::Map<Mat4>(mvp.data()) = model;
        return mvp;
    }

    std::array<R, 16> matV(const int* vp, bool offsetDepth = false, bool colmajor = false) const {
        R trans[] = { (bbox[0] + bbox[dim]) / 2, (bbox[1] + bbox[1 + dim]) / 2, (bbox[2] + bbox[2 + dim]) / 2 };
        R ss = drawscale();
        R t[] = { mTranslate[0] - trans[0], mTranslate[1] - trans[1], mTranslate[2] - trans[2] };

        using Mat4 = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;

        float up[] = { 0, 1, 0 };// Head is up (set to 0,-1,0 to look upside-down)
        Mat4 view = lookAt(campos, mViewCenter, up);

        std::array<R, 16> mvp;
        Eigen::Map<Mat4>(mvp.data()) = view;
        return mvp;
    }

    std::array<R, 16> matP(const int* vp, bool offsetDepth = false, bool colmajor = false) const {
        R trans[] = { (bbox[0] + bbox[dim]) / 2, (bbox[1] + bbox[1 + dim]) / 2, (bbox[2] + bbox[2 + dim]) / 2 };
        R ss = drawscale();
        R t[] = { mTranslate[0] - trans[0], mTranslate[1] - trans[1], mTranslate[2] - trans[2] };

        using Mat4 = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;

        Mat4 proj = perspective(35.f, vp[2] / float(vp[3]), 1.5f, 100.0f);

        std::array<R, 16> mvp;
        Eigen::Map<Mat4>(mvp.data()) = proj;
        return mvp;
    }

    static void boundingbox(const std::vector<vec3> x, int nv, R* bbox)
    {
        if (nv < 1) {
            printf("empty point set!\n");
            return;
        }
        bbox[0] = bbox[0 + dim] = x[0][0];
        bbox[1] = bbox[1 + dim] = x[0][1];
        bbox[2] = bbox[2 + dim] = x[0][2];


        for (int i = 1; i < nv; i++) {
            bbox[0] = std::min(bbox[0], x[i][0]);
            bbox[0 + dim] = std::max(bbox[0 + dim], x[i][0]);

            bbox[1] = std::min(bbox[1], x[i][1]);
            bbox[1 + dim] = std::max(bbox[1 + dim], x[i][1]);

            bbox[2] = std::min(bbox[2], x[i][2]);
            bbox[2 + dim] = std::max(bbox[2 + dim], x[i][2]);
        }
    }
    // // // ///////////////////////////////////////////////////////
    void moveInScreen(int x0, int y0, int x1, int y1, int* vp) {
        float dx = (x1 - x0) * 2.f / vp[2];
        float dy = -(y1 - y0) * 2.f / vp[3];

        mViewCenter[0] -= dx;
        mViewCenter[1] -= dy;
    }

    // //////////////////////////////////////////////////////////////
    void updateBBox() {
        boundingbox(control_points.vertices, nVertex, bbox);
    }

    void upload()
    {
        nVertex = control_points.nVertex();
        nFace = control_points.nFace();
        //rasterizer.gpu_vertices.set_vertices(control_points.vertices);
        //rasterizer.gpu_vertices.set_clc_mat(control_points.vertices);
        //rasterizer.gpu_vertices.set_clc_matrix(control_points.vertices);
        //rasterizer.gpu_vertices.set_adj_faces(control_points.adjfaces);
        updateBBox();
    }

    void alloc() {

        glGenVertexArrays(1, &bezierVAO);
        glBindVertexArray(bezierVAO);

        glGenBuffers(1, &bezierVBO);
        glBindBuffer(GL_ARRAY_BUFFER, bezierVBO);
        glBufferData(GL_ARRAY_BUFFER, 3 * control_points.vertices.size() * sizeof(float), nullptr, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);
        glBufferSubData(GL_ARRAY_BUFFER, 0, (3 * control_points.vertices.size()) * sizeof(float), control_points.vertices.data());

        glGenBuffers(1, &bezierVBO2);
        glBindBuffer(GL_ARRAY_BUFFER, bezierVBO2);
        glBufferData(GL_ARRAY_BUFFER, control_points.showface.size() *sizeof(float), nullptr, GL_STATIC_DRAW);
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);
        glBufferSubData(GL_ARRAY_BUFFER, 0, control_points.showface.size() * sizeof(float), control_points.showface.data());

        glGenBuffers(1, &bezierVBO3);
        glBindBuffer(GL_ARRAY_BUFFER, bezierVBO3);
        glBufferData(GL_ARRAY_BUFFER, control_points.faceID.size() * sizeof(float), nullptr, GL_STATIC_DRAW);
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(2);
        glBufferSubData(GL_ARRAY_BUFFER, 0, control_points.faceID.size() * sizeof(float), control_points.faceID.data());

        glBindVertexArray(0);
    }

    void alloc_compute_buffers() {

        glGenBuffers(1, &control_points_buffer);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, control_points_buffer);
        glBufferData(GL_SHADER_STORAGE_BUFFER, 3 * control_points.vertices.size() * sizeof(float), control_points.vertices.data(), GL_DYNAMIC_DRAW);

        glGenBuffers(1, &adj_face_buffer);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, adj_face_buffer);
        glBufferData(GL_SHADER_STORAGE_BUFFER, 3 * control_points.nFace() * sizeof(int), control_points.adjfaces.data(), GL_DYNAMIC_DRAW);

        glGenBuffers(1, &world_position_buffer);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, world_position_buffer);
        glBufferData(GL_SHADER_STORAGE_BUFFER, 4 * control_points.vertices.size() * sizeof(float), nullptr, GL_DYNAMIC_DRAW);

        glGenBuffers(1, &screen_position_buffer);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, screen_position_buffer);
        glBufferData(GL_SHADER_STORAGE_BUFFER, 2 * control_points.vertices.size() * sizeof(float), nullptr, GL_DYNAMIC_DRAW);

        glGenBuffers(1, &coeff_buffer);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, coeff_buffer);
        glBufferData(GL_SHADER_STORAGE_BUFFER, 70 * control_points.nFace() * sizeof(float), nullptr, GL_DYNAMIC_DRAW);

        glGenBuffers(1, &should_redraw_buffer);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, should_redraw_buffer);
        glBufferData(GL_SHADER_STORAGE_BUFFER, control_points.nFace() * sizeof(int), nullptr, GL_DYNAMIC_DRAW);

    }

    void create_tex(const int* vp) {
        samplingFBO.deleteFbo();
        samplingFBO.createFbo(vp[2], vp[3]);

        fbo.deleteFbo();
        fbo.createFbo(vp[2], vp[3]);
    }

    void draw_ras(const int* vp) {
        // ========== 1. Render full scene to framebuffer ==========
        samplingFBO.bind();
        GLenum draw_bufs[1] = { GL_COLOR_ATTACHMENT0 };
        glDrawBuffers(1, draw_bufs);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);

        glBindVertexArray(bezierVAO);
        glPatchParameteri(GL_PATCH_VERTICES, 15);

        prog_soft_ras.bind();
        prog_soft_ras.setUniform("model", matM(vp).data());
        prog_soft_ras.setUniform("view", matV(vp).data());
        prog_soft_ras.setUniform("projection", matP(vp).data());
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, should_redraw_buffer);

        glDrawArrays(GL_PATCHES, 0, control_points.vertices.size());
        prog_soft_ras.unbind();

        glBindFramebuffer(GL_FRAMEBUFFER, 0); // Unbind FBO


        // ========== 2. Run compute shader to analyze patches ==========
        prog_compute.bind();
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, screen_position_buffer);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, world_position_buffer);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, control_points_buffer);
        prog_compute.setUniform("MVP", matMVP(vp).data());
        prog_compute.setUniform("width", float(vp[2]));
        prog_compute.setUniform("height", float(vp[3]));
        glDispatchCompute((control_points.vertices.size() + 255) / 256, 1, 1);
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
        prog_compute.unbind();


        // ========== 3. Compute coefficients ==========
        prog_compute_coeff.bind();
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, coeff_buffer);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, screen_position_buffer);
        glDispatchCompute((control_points.nFace() + 255) / 256, 1, 1);
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
        prog_compute_coeff.unbind();

        // ========== should_redraw_buffer ==========
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, should_redraw_buffer);
        glClearBufferData(
            GL_SHADER_STORAGE_BUFFER,
            GL_R32I,
            GL_RED_INTEGER,
            GL_INT,
            nullptr            
        );
        // ========== 4. Compute iteration to decide should_redraw ==========
        prog_compute_iter.bind();
        glActiveTexture(GL_TEXTURE0);
        glBindImageTexture(0, samplingFBO.gNormalAndDepth, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, world_position_buffer);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, coeff_buffer);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, adj_face_buffer);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, should_redraw_buffer);
        glDispatchCompute((vp[2] + 7) / 8, (vp[3] + 7) / 8, 1);  // Assuming 8x8 tile
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
        prog_compute_iter.unbind();


        // ========== 5. Selective patch recovery based on should_redraw ==========
        if(show_contour){
            samplingFBO.bind();
            GLenum draw_bufs2[1] = { GL_COLOR_ATTACHMENT1 };
            glDrawBuffers(1, draw_bufs2);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_CULL_FACE);

            glBindVertexArray(bezierVAO);
            glPatchParameteri(GL_PATCH_VERTICES, 15);

            prog_recovery.bind();
            prog_recovery.setUniform("MVP", matMVP(vp).data());
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, should_redraw_buffer);

            glBindVertexArray(bezierVAO);
            glDrawArrays(GL_PATCHES, 0, control_points.vertices.size());
            glBindVertexArray(0);
            prog_recovery.unbind();
            glBindFramebuffer(GL_FRAMEBUFFER, 0);

            prog_compute_recovery.bind();
            glActiveTexture(GL_TEXTURE0);
            glBindImageTexture(0, samplingFBO.gNormalAndDepth, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
            glActiveTexture(GL_TEXTURE1);
            glBindImageTexture(1, samplingFBO.compute_texture, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, world_position_buffer);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, coeff_buffer);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, adj_face_buffer);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, should_redraw_buffer);
            glDispatchCompute((vp[2] + 7) / 8, (vp[3] + 7) / 8, 1);  // Assuming 8x8 tile
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
            prog_compute_recovery.unbind();
        }
        
        // ========== 6. Final display pass ==========
        prog_screen.bind();
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, samplingFBO.gNormalAndDepth);
        prog_screen.setUniform("gNormalAndDepth", 0);
        prog_screen.setUniform("show_patch", int(show_patch));
        renderQuad();  // Render fullscreen quad
        prog_screen.unbind();
    }

    void renderQuad()
    {
        if (quadVAO == 0)
        {
            float quadVertices[] = {
                // positions        // texture Coords
                -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
                -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
                 1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
                 1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
            };

            // setup plane VAO
            glGenVertexArrays(1, &quadVAO);
            glGenBuffers(1, &quadVBO);
            glBindVertexArray(quadVAO);
            glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
        }
        glBindVertexArray(quadVAO);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindVertexArray(0);
    }

    static void buildShaders() {

        prog_soft_ras.compileShaderFromFile("shader/sampling_vert.glsl", GLProgram::ShaderType::VERTEX_SHADER);
        prog_soft_ras.compileShaderFromFile("shader/sampling_frag.glsl", GLProgram::ShaderType::FRAGMENT_SHADER);
        prog_soft_ras.compileShaderFromFile("shader/sampling_tcs.glsl", GLProgram::ShaderType::TESS_CONTROL_SHADER);
        prog_soft_ras.compileShaderFromFile("shader/sampling_tes.glsl", GLProgram::ShaderType::TESS_EVALUATION_SHADER);
        prog_soft_ras.link();
        prog_soft_ras.bind();

        prog_recovery.compileShaderFromFile("recovery_shader/recovery_vert.glsl", GLProgram::ShaderType::VERTEX_SHADER);
        prog_recovery.compileShaderFromFile("recovery_shader/recovery_frag.glsl", GLProgram::ShaderType::FRAGMENT_SHADER);
        prog_recovery.compileShaderFromFile("recovery_shader/recovery_tcs.glsl", GLProgram::ShaderType::TESS_CONTROL_SHADER);
        prog_recovery.compileShaderFromFile("recovery_shader/recovery_tes.glsl", GLProgram::ShaderType::TESS_EVALUATION_SHADER);
        prog_recovery.link();
        prog_recovery.bind();

        prog_compute.compileShaderFromFile("shader/compute_vt.glsl", GLProgram::ShaderType::COMPUTE_SHADER);
        prog_compute.link();
        prog_compute.bind();

        prog_compute_coeff.compileShaderFromFile("shader/compute_coeff.glsl", GLProgram::ShaderType::COMPUTE_SHADER);
        prog_compute_coeff.link();
        prog_compute_coeff.bind();

        prog_compute_iter.compileShaderFromFile("shader/compute_iter.glsl", GLProgram::ShaderType::COMPUTE_SHADER);
        prog_compute_iter.link();
        prog_compute_iter.bind();

        prog_compute_recovery.compileShaderFromFile("shader/compute_recovery.glsl", GLProgram::ShaderType::COMPUTE_SHADER);
        prog_compute_recovery.link();
        prog_compute_recovery.bind();

        prog_screen.compileAndLinkAllShadersFromFile("shader/screen.vs", "shader/screen.fs");
    }
};

typedef GLBezierT<> MyBezierT;