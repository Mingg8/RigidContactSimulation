#pragma once

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include "utils.hh"

class Object {
    private:

    public:
    int indices_size;
    GLuint vertexbuffer, colorbuffer, normalbuffer, elementbuffer;
    GLuint ambcolorbuffer, spccolorbuffer;
    GLuint alphabuffer;
    GLuint vertexarrayobject;

    Object();
    Object(const Object&) = delete;
    ~Object() = default;

    void drawElements(glm::mat4 MVP, glm::mat4 Model, glm::mat4 ViewMatrix,
        glm::mat4 ProjectionMatrix, GLuint MatrixID, GLuint ModelMatrixID,
        GLuint ViewMatrixID, GLuint ProjectionMatrixID, GLuint LightID,
        glm::vec3 lightPos);
    void drawElements_franka(glm::mat4 MVP, glm::mat4 Model, glm::mat4 ViewMatrix,
        glm::mat4 ProjectionMatrix, GLuint MatrixID, GLuint ModelMatrixID,
        GLuint ViewMatrixID, GLuint ProjectionMatrixID, GLuint LightID,
        glm::vec3 lightPos);
    void drawArrays(glm::mat4 MVP, glm::mat4 Model, glm::mat4 ViewMatrix,
        glm::mat4 ProjectionMatrix, GLuint MatrixID, GLuint ModelMatrixID,
        GLuint ViewMatrixID, GLuint ProjectionMatrixID,
        GLuint LightID, glm::vec3 lightPos, vector<glm::vec3> &data,
        vector<glm::vec3> &normal, vector<glm::vec3> &color,
        vector<glm::vec3> &spccolor);
    void drawLines(glm::mat4 MVP, glm::mat4 Model, glm::mat4 ViewMatrix,
        glm::mat4 ProjectionMatrix, GLuint MatrixID, GLuint ModelMatrixID,
        GLuint ViewMatrixID, GLuint ProjectionMatrixID, GLuint LightID,
        glm::vec3 lightPos, vector<glm::vec3> &data, vector<glm::vec3> &color,
        vector<glm::vec3> &spccolor);
    void setElement(vector<glm::vec3>&, vector<glm::vec3>&, vector<glm::vec3>&,
        vector<unsigned short>&, vector<glm::vec3>&);
    void setElement_franka(vector<glm::vec3>&, vector<glm::vec3>&, 
        vector<glm::vec3>&, vector<glm::vec3>&, vector<glm::vec3>&,
        vector<unsigned int>&, vector<glm::vec3>&);
    void setBuffer(int _size);
    void cleanUp();
};