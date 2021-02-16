#include "object.hh"

Object::Object() {
}

void Object::drawElements(glm::mat4 MVP, glm::mat4 Model, glm::mat4 ViewMatrix,
    glm::mat4 ProjectionMatrix, GLuint MatrixID, GLuint ModelMatrixID,
    GLuint ViewMatrixID, GLuint ProjectionMatrixID, GLuint LightID,
    glm::vec3 lightPos) {

    glBindVertexArray(vertexarrayobject);
    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
    glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &Model[0][0]);
    glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);
    glUniformMatrix4fv(ProjectionMatrixID, 1, GL_FALSE, &ProjectionMatrix[0][0]);

    // Draw the triangles !
    glDrawElements(GL_TRIANGLES, indices_size*3, GL_UNSIGNED_SHORT, (void*)0);
    glBindVertexArray(0);
}


void Object::drawElements_franka(glm::mat4 MVP, glm::mat4 Model, glm::mat4 ViewMatrix,
    glm::mat4 ProjectionMatrix, GLuint MatrixID, GLuint ModelMatrixID, 
    GLuint ViewMatrixID, GLuint ProjectionMatrixID, GLuint LightID,
    glm::vec3 lightPos) {
    glBindVertexArray(vertexarrayobject);
    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
    glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &Model[0][0]);
    glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);
    glUniformMatrix4fv(ProjectionMatrixID, 1, GL_FALSE, &ProjectionMatrix[0][0]);

    glDrawElements(GL_TRIANGLES, indices_size*3, GL_UNSIGNED_INT, (void*)0);
    glBindVertexArray(0);
}


void Object::drawArrays(glm::mat4 MVP, glm::mat4 Model, glm::mat4 ViewMatrix,
    glm::mat4 ProjectionMatrix, GLuint MatrixID, GLuint ModelMatrixID,
    GLuint ViewMatrixID, GLuint ProjectionMatrixID, GLuint LightID,
    glm::vec3 lightPos, vector<glm::vec3> &data, vector<glm::vec3> &normal,
    vector<glm::vec3> &color, vector<glm::vec3> &spccolor) 
{
    glBindVertexArray(vertexarrayobject);
    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
    glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &Model[0][0]);
    glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);
    glUniformMatrix4fv(ProjectionMatrixID, 1, GL_FALSE, &ProjectionMatrix[0][0]);
    
    // 1rst attribute buffer : vertices
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(glm::vec3), &data[0],
        GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    // attribute / size / type / normalized? // stride // array buffer offset

    // 2nd attribute buffer : Color
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glBufferData(GL_ARRAY_BUFFER, color.size() * sizeof(glm::vec3), &color[0],
        GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

    // 3rd attribute buffer : normals
    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
    glBufferData(GL_ARRAY_BUFFER, normal.size() * sizeof(glm::vec3), &normal[0],
        GL_STATIC_DRAW);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

    // 2nd attribute buffer : spccolor
    glEnableVertexAttribArray(3);
    glBindBuffer(GL_ARRAY_BUFFER, spccolorbuffer);
    glBufferData(GL_ARRAY_BUFFER, color.size() * sizeof(glm::vec3), &color[0],
        GL_STATIC_DRAW);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

    // 2nd attribute buffer : ambcolor
    glEnableVertexAttribArray(4);
    glBindBuffer(GL_ARRAY_BUFFER, ambcolorbuffer);
    glBufferData(GL_ARRAY_BUFFER, color.size() * sizeof(glm::vec3), &color[0],
        GL_STATIC_DRAW);
    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);


    // 2nd attribute buffer : alpha
    glEnableVertexAttribArray(5);
    glBindBuffer(GL_ARRAY_BUFFER, alphabuffer);
    glBufferData(GL_ARRAY_BUFFER, spccolor.size() * sizeof(glm::vec3), &spccolor[0],
        GL_STATIC_DRAW);
    glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

    // Draw the triangles !
    glDrawArrays(GL_TRIANGLES, 0, data.size());
    // mode / count / type / element array buffer offset
    glBindVertexArray(0);
}

void Object::drawLines(glm::mat4 MVP, glm::mat4 Model, glm::mat4 ViewMatrix,
        glm::mat4 ProjectionMatrix, GLuint MatrixID, GLuint ModelMatrixID,
        GLuint ViewMatrixID, GLuint ProjectionMatrixID,
        GLuint LightID, glm::vec3 lightPos, vector<glm::vec3> &data, 
        vector<glm::vec3> &color, vector<glm::vec3> &spccolor)
{
    glBindVertexArray(vertexarrayobject);
    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
    glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &Model[0][0]);
    glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);
    glUniformMatrix4fv(ProjectionMatrixID, 1, GL_FALSE, &ProjectionMatrix[0][0]);
    
    // 1rst attribute buffer : vertices
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(glm::vec3), &data[0],
        GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    // attribute / size / type / normalized? // stride // array buffer offset

    // 2nd attribute buffer : Color
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glBufferData(GL_ARRAY_BUFFER, color.size() * sizeof(glm::vec3), &color[0],
        GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

    // Draw the triangles !
    glDrawArrays(GL_LINES, 0, data.size());
    // mode / count / type / element array buffer offset
    glBindVertexArray(0);
}

void Object::setElement(vector<glm::vec3> &indexed_vertices, 
    vector<glm::vec3> &indexed_color, vector<glm::vec3> &indexed_normals,
    vector<unsigned short> &indices, vector<glm::vec3> &indexed_spccolor)
{
    // VBO generation
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_vertices.size() * sizeof(glm::vec3),
        &indexed_vertices[0], GL_STATIC_DRAW);

	glGenBuffers(1, &colorbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_color.size() * sizeof(glm::vec3),
         &indexed_color[0], GL_STATIC_DRAW);

	glGenBuffers(1, &normalbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_normals.size() * sizeof(glm::vec3),
        &indexed_normals[0], GL_STATIC_DRAW);

	glGenBuffers(1, &spccolorbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, spccolorbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_color.size() * sizeof(glm::vec3),
        &indexed_color[0], GL_STATIC_DRAW);

	glGenBuffers(1, &ambcolorbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, ambcolorbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_color.size() * sizeof(glm::vec3),
        &indexed_color[0], GL_STATIC_DRAW);

	glGenBuffers(1, &alphabuffer);
	glBindBuffer(GL_ARRAY_BUFFER, alphabuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_spccolor.size() * sizeof(glm::vec3),
        &indexed_spccolor[0], GL_STATIC_DRAW);

    // VAO generation
    glGenVertexArrays(1, &vertexarrayobject);
    glBindVertexArray(vertexarrayobject);

	// Generate a buffer for the indices as well
	glGenBuffers(1, &elementbuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(indices[0]),
        &indices[0] , GL_STATIC_DRAW);

    ///////////////////////
    // 1rst attribute buffer : vertices
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);
    // attribute / size / type / normalized? // stride // array buffer offset

    // 2nd attribute buffer : UVs
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(1);

    // 3rd attribute buffer : normals
    glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(2);

    // 2nd attribute buffer : UVs
    glBindBuffer(GL_ARRAY_BUFFER, spccolorbuffer);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(3);

    // 2nd attribute buffer : UVs
    glBindBuffer(GL_ARRAY_BUFFER, ambcolorbuffer);
    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(4);

    // 2nd attribute buffer : alpha
    glBindBuffer(GL_ARRAY_BUFFER, alphabuffer);
    glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(5);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void Object::setElement_franka(vector<glm::vec3> &indexed_vertices, 
    vector<glm::vec3> &indexed_color, vector<glm::vec3> &indexed_normals, 
    vector<glm::vec3> &indexed_color_a,  vector<glm::vec3> &indexed_color_s,
    vector<unsigned int> &indices, vector<glm::vec3> &indexed_alpha) 
{
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_vertices.size() * sizeof(indexed_vertices[0]),
        &indexed_vertices[0], GL_STATIC_DRAW);

	glGenBuffers(1, &colorbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_color.size() * sizeof(glm::vec3),
         &indexed_color[0], GL_STATIC_DRAW);

	glGenBuffers(1, &normalbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_normals.size() * sizeof(glm::vec3),
        &indexed_normals[0], GL_STATIC_DRAW);

	glGenBuffers(1, &spccolorbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, spccolorbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_color_s.size() * sizeof(glm::vec3),
         &indexed_color_s[0], GL_STATIC_DRAW);

	glGenBuffers(1, &ambcolorbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, ambcolorbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_color_a.size() * sizeof(glm::vec3),
         &indexed_color_a[0], GL_STATIC_DRAW);

	glGenBuffers(1, &alphabuffer);
	glBindBuffer(GL_ARRAY_BUFFER, alphabuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_alpha.size() * sizeof(glm::vec3),
         &indexed_alpha[0], GL_STATIC_DRAW);

    glGenVertexArrays(1, &vertexarrayobject);
    glBindVertexArray(vertexarrayobject);

	// Generate a buffer for the indices as well
	glGenBuffers(1, &elementbuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(indices[0]),
        &indices[0] , GL_STATIC_DRAW);

    //////////////////////////////
    // 1rst attribute buffer : vertices
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);
    // attribute / size / type / normalized? // stride // array buffer offset

    // 2nd attribute buffer : UVs
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(1);

    // 3rd attribute buffer : normals
    glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(2);

    // 3rd attribute buffer : spc color
    glBindBuffer(GL_ARRAY_BUFFER, spccolorbuffer);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(3);

    // 3rd attribute buffer : amb color
    glBindBuffer(GL_ARRAY_BUFFER, ambcolorbuffer);
    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(4);

    // 3rd attribute buffer : amb color
    glBindBuffer(GL_ARRAY_BUFFER, alphabuffer);
    glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(5);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void Object::cleanUp() {
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteBuffers(1, &colorbuffer);
	glDeleteBuffers(1, &normalbuffer);
	glDeleteBuffers(1, &elementbuffer);
}

void Object::setBuffer(int _size) {
    glGenVertexArrays(1, &vertexarrayobject);
    glBindVertexArray(vertexarrayobject);
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glGenBuffers(1, &normalbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
    glGenBuffers(1, &colorbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glGenBuffers(1, &spccolorbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, spccolorbuffer);
    glGenBuffers(1, &ambcolorbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, ambcolorbuffer);
    glGenBuffers(1, &alphabuffer);
    glBindBuffer(GL_ARRAY_BUFFER, alphabuffer);
	glBindVertexArray(0);
    indices_size = _size;
}