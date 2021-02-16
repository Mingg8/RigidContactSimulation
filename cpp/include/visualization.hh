#pragma once
#include "utils.hh"
#include "object.hh"
#include "tiny_obj_loader.h"

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include <common/shader.hpp>
#include <common/texture.hpp>
#include <common/objloader.hpp>
#include <common/vboindexer.hpp>


static vector<glm::vec3> DEFAULT_VECTOR;

class Visualization {
  private:
    unique_ptr<Object> male, floor, axis, franka[8], sphere, ATI, 
      ATI2, female_bottom;
    vector<unique_ptr<Object>> female;
    // GLuint vertexbuffer_f[4], normalbuffer_f[4], colorbuffer_f[4];
    // GLuint colorbuffer_floor;

    GLuint MatrixID, ModelMatrixID, ViewMatrixID, ProjectionMatrixID;
	  GLuint VertexArrayID;
    GLuint Texture, TextureID, LightID;

    glm::vec3 cam_pos, obj_pos;
    float horizontalAngle, verticalAngle, speed, FoV;

    glm::mat4 ProjectionMatrix, ViewMatrix;
    glm::vec3 direction, right, up;
    bool mouseMovePressed, mouseRotatePressed, mouseZoomPressed;
    double lastZoom, lastX, lastY, beginU, beginV;
    int width, height;

    int setGLFW();
    void setProgram();
    // void framebufferSizeCallback(GLFWwindow* window, int _width, int _height);

  public:
    enum OBJ {MALE, FEMALE, FLOOR, AXIS, FRANKA, SPHERE,
      FT, FT2, FEMALE_BOTTOM};
    GLFWwindow* window;
    GLuint programID;
    Visualization(int _obj_num);
    void setView(glm::vec3 pos, glm::vec3 obj_pos, float hAngle, float vAngle, 
        float iFoV, float _speed);
    void mouseCB();
    void controlView(float deltaTime);
    void setArrays(int floor_size);
    void drawObjects(OBJ obj_type, glm::mat4 Model, glm::vec3 lightPos,
        vector<glm::vec3> &data = DEFAULT_VECTOR,
        vector<glm::vec3> &normal = DEFAULT_VECTOR,
        vector<glm::vec3> &color = DEFAULT_VECTOR,
        vector<glm::vec3> &spccolor = DEFAULT_VECTOR,
        int ind = -1);
    void setHexa(vector<glm::vec3>&hexa_pnts, vector<unsigned short>&indices,
      vector<glm::vec3> &hexa_color, vector<glm::vec3> &hexa_normal, 
      vector<glm::vec3> &hexa_spccolor);

    void cleanUp();
    void readObj(const char * path, glm::vec3 &color, float alpha, OBJ obj);
    void setFranka();
    void drawFranka(glm::vec3 lightPos, int nbFrames);
    int obj_num;
    void load_franka(float alpha);
    void load_ATI(float alpha);
    void load_ATI2(float alpha);
};