#ifndef VIEWER_H
#define VIEWER_H

#include "opengl.h"
#include "shader.h"
#include "camera.h"
#include "mesh.h"
#include "sphere.h"
#include "fbo.h"
#include "quad.h"

#include <iostream>

class Viewer{
public:
    //! Constructor
    Viewer();
    virtual ~Viewer();

    // gl stuff
    void init(int w, int h);
    void display();
    void updateScene();
    void reshape(int w, int h);
    void loadProgram();

    // test
    void saveColorPicture();
    void saveNormalPicture();

    // events
    void mousePressed(GLFWwindow* window, int button, int action);
    void mouseMoved(int x, int y);
    void mouseScroll(double x, double y);
    void keyPressed(int key, int action, int mods);
    void charPressed(int key);

private:
    int _winWidth, _winHeight;

    Camera _cam;
    Shader _blinnPrg, _simplePrg, _gbufferPrg;

    // some geometry to render
    std::vector<Shape*> _shapes;
    std::vector<float> _specularCoef;

    // geometrical representation of a pointlight
    Sphere _pointLight;
    Eigen::Vector3f _lightColor;
    float _lightAngle = 0.f;

    std::vector<Sphere*> _lights;
    std::vector<Eigen::Vector3f*> _lightsColors;

    // Mouse parameters
    Eigen::Vector2f _lastMousePos;
    int _button = -1;

    FBO _framebufferO;
    Quad* _quadGBuff;
};

#endif
