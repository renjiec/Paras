#define _CRT_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS

#define FREEGLUT_STATIC

#include <glad/glad.h>
#include <GL/glut.h>
#include <GL/freeglut_ext.h>

#define TW_STATIC
#include <AntTweakBar.h>

#include <fstream>
#include <ctime>
#include <memory>
#include <vector>
#include <string>
#include <cstdlib>
#include <iostream>
#include "glprogram.h"
#include "soft_ras/vaobezierT.h"
#include "trackball.h"

GLProgram MyBezierT::prog_recovery;
GLProgram MyBezierT::prog_soft_ras;
GLProgram MyBezierT::prog_screen;

// compute shader programs
GLProgram MyBezierT::prog_compute;
GLProgram MyBezierT::prog_compute_coeff;
GLProgram MyBezierT::prog_compute_iter;
GLProgram MyBezierT::prog_compute_recovery;

MyBezierT M;

int viewport[4] = { 0, 0, 1920, 1080 };

int mousePressButton;
int mouseButtonDown;
int mousePos[2];

float fps = 0;
bool a = true;
bool showTwBar = true;

void display()
{
    static clock_t lastTime = 0;
    static int numRenderedFrames = 0;
    const int numFramesPerReport = 250;
    if ((numRenderedFrames + 1) % numFramesPerReport == 0) {
        clock_t now = clock();
        float dual = (now - lastTime) / float(CLOCKS_PER_SEC);
        fps = numFramesPerReport / dual;
        M.fps = fps;
        lastTime = now;
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_MULTISAMPLE);

    glViewport(0, 0, viewport[2], viewport[3]);

    M.draw_ras(viewport);
    if (showTwBar) {
        TwDraw();
    }
    glutSwapBuffers();
    numRenderedFrames++;
}

void onMouseButton(int button, int updown, int x, int y)
{
    if (!TwEventMouseButtonGLUT(button, updown, x, y)) {
        mousePressButton = button;
        mouseButtonDown = updown;

        mousePos[0] = x;
        mousePos[1] = y;
    }

    glutPostRedisplay();
}

void onMouseMove(int x, int y)
{
    if (!TwEventMouseMotionGLUT(x, y)) {
        if (mouseButtonDown == GLUT_DOWN) {
            if (mousePressButton == GLUT_MIDDLE_BUTTON) {
                M.moveInScreen(mousePos[0], mousePos[1], x, y, viewport);
            }
            else if (mousePressButton == GLUT_LEFT_BUTTON) {
                const float s[2] = { 2.f / viewport[2], 2.f / viewport[3] };
                auto r = Quat<float>(M.mQRotate) * Quat<float>::trackball(x * s[0] - 1, 1 - y * s[1], s[0] * mousePos[0] - 1, 1 - s[1] * mousePos[1]);
                std::copy_n(r.q, 4, M.mQRotate);
            }
        }
    }
    mousePos[0] = x; mousePos[1] = y;
    glutPostRedisplay();
}

void onMouseWheel(int wheel_number, int direction, int x, int y)
{
    M.mMeshScale *= direction > 0 ? 1.1f : 0.9f;
    glutPostRedisplay();
}

void onKeyboard(unsigned char code, int x, int y)
{
    if (!TwEventKeyboardGLUT(code, x, y)) {
        switch (code) {
        case 17:
            exit(0);
        case 'f':
            glutFullScreenToggle();
            break;
        case 'b':
            break;
        }
    }
    glutPostRedisplay();
}

int initGL(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_MULTISAMPLE);
    glutInitWindowSize(viewport[2], viewport[3]);
    glutCreateWindow(argv[0]);

    // !Load the OpenGL functions. after the opengl context has been created
    if (gladLoadGL() == 0)
        return -1;

    glClearColor(1.f, 1.f, 1.f, 1.f);

    // re-allco
    glutReshapeFunc([](int w, int h) { viewport[2] = w; viewport[3] = h; glViewport(0, 0, w, h); TwWindowSize(w, h); M.create_tex(viewport); });
    glutDisplayFunc(display);
    glutKeyboardFunc(onKeyboard);
    glutIdleFunc([]() {  glutPostRedisplay(); });
    glutMouseFunc(onMouseButton);
    glutMotionFunc(onMouseMove);
    glutMouseWheelFunc(onMouseWheel);
    glutCloseFunc([]() {TwTerminate();  });
    return 0;
}

void createTweakbar()
{
    TwBar* bar = TwGetBarByName("Bezier_Surfaces_Rendering");
    if (bar)    TwDeleteBar(bar);

    //Create a tweak bar
    bar = TwNewBar("Bezier_Surfaces_Rendering");
    TwDefine(" Bezier_Surfaces_Rendering size='250 150' color='0 128 255' text=light alpha=128 position='5 5'"); // change default tweak bar size and color

    TwAddVarRO(bar, "#Bezier Patch", TW_TYPE_INT32, &M.nFace, " group='Surface Info'");
    TwAddVarRO(bar, "#Control Point", TW_TYPE_INT32, &M.nVertex, " group='Surface Info'");
    TwAddVarRO(bar, "FPS:", TW_TYPE_FLOAT, &M.fps, " group='Surface Info'");

    TwAddVarRW(bar, "contour_recovery", TW_TYPE_BOOLCPP, &M.show_contour, " group='Rendering Mode'");
    TwAddVarRW(bar, "show_patch", TW_TYPE_BOOLCPP, &M.show_patch, " group='Rendering Mode'");
}

int main(int argc, char *argv[])
{
    if (initGL(argc, argv)) {
        fprintf(stderr, "!Failed to initialize OpenGL!Exit...");
        exit(-1);
    }

    MyBezierT::buildShaders();

    if (!M.loadFromBinaryFile("data_benching\\bigguy_data.bin")) {
        std::cerr << "Failed to load Bezier data." << std::endl;
        return -1;
    }

    M.upload();
    M.alloc();
    M.create_tex(viewport);
    M.alloc_compute_buffers();

    // create a Twbar
    TwInit(TW_OPENGL_CORE, NULL);
    TwGLUTModifiersFunc(glutGetModifiers);
    glutSpecialFunc([](int key, int x, int y) { TwEventSpecialGLUT(key, x, y); glutPostRedisplay(); }); // important for special keys like UP/DOWN/LEFT/RIGHT ...
    TwCopyStdStringToClientFunc([](std::string& dst, const std::string& src) {dst = src; });
    createTweakbar();

    glutIdleFunc([]() {
        glutPostRedisplay();
        }
    );

    glutMainLoop();
    
    return 0;
}
