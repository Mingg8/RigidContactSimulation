#include "visualization.hh"

Visualization::Visualization(int _obj_num) 
: obj_num(_obj_num) {
    setGLFW();
    setProgram();    
};


int Visualization::setGLFW() {
	// Initialise GLFW
	if( !glfwInit() )
	{
		fprintf( stderr, "Failed to initialize GLFW\n" );
		getchar();
		return -1;
	}
    width = 1024;
    height = 768;

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // glfwWindowHint(GLFW_DOUBLEBUFFER, GL_FALSE);

	// Open a window and create its OpenGL context
	window = glfwCreateWindow( width, height, "Snapconnector Simulation", NULL, NULL);
	if( window == NULL ){
		fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
		getchar();
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return -1;
	}

	glClearColor(0.9f, 0.9f, 0.9f, 0.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);
};


void Visualization::setProgram()
{
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);
	programID = LoadShaders( "../src/StandardShading.vertexshader", "../src/StandardShading.fragmentshader");
	MatrixID = glGetUniformLocation(programID, "MVP");
	ViewMatrixID = glGetUniformLocation(programID, "V");
	ModelMatrixID = glGetUniformLocation(programID, "M");
    ProjectionMatrixID = glGetUniformLocation(programID, "P");

	// Get a handle for our "LightPosition" uniform
	glUseProgram(programID);
	LightID = glGetUniformLocation(programID, "LightPosition_worldspace");
}

void Visualization::setView(glm::vec3 pos, glm::vec3 _obj_pos, float hAngle, float vAngle, 
    float iFoV, float _speed) {
    cam_pos = pos;
    obj_pos = _obj_pos;
    horizontalAngle = hAngle;
    verticalAngle =vAngle;
    FoV = iFoV;
    speed = _speed;
        
	ProjectionMatrix = glm::perspective(glm::radians(FoV), 
        float(width) / float(height), 0.1f, 100.0f);
	direction = glm::vec3(cos(verticalAngle)*sin(horizontalAngle), 
        sin(verticalAngle), cos(verticalAngle)*cos(horizontalAngle));
	right = glm::vec3(sin(horizontalAngle - PI / 2), 0, cos(horizontalAngle - PI / 2));
	ViewMatrix = glm::lookAt(cam_pos, cam_pos + direction, up);
}

void Visualization::mouseCB() {
    double x, y;
    glfwGetCursorPos(window, &x, &y);
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
            // move
            if (mouseMovePressed == false) {
                lastX = x;
                lastY = y;
            }
            mouseMovePressed = true;
            mouseRotatePressed = false;
            mouseZoomPressed = false;
            double move_x, move_y;
            move_x = (x-lastX)/width*0.01;
            move_y = (y-lastY)/width*0.01;
            cam_pos = cam_pos + (float)move_x * right + (float)move_y * up;

        } else if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
            if (mouseZoomPressed == false) {
                lastZoom = y;
            }
            mouseMovePressed = false;
            mouseRotatePressed = false;
            mouseZoomPressed = true;
            cam_pos -= direction * (float)((y - lastZoom)/height) * 0.1f;
        } else {
            // rotate camera
            if (mouseRotatePressed == false) {
                beginU = x;
                beginV = y;
            }
            mouseMovePressed = false;
            mouseRotatePressed = true;
            mouseZoomPressed = false;
            double move_u = (x - beginU)/width * 0.1;
            double move_v = (y - beginV)/height * 0.1;
            horizontalAngle -= (float)move_u;
            verticalAngle += (float)move_v;
            glm::vec3 cam_to_obj = cam_pos - obj_pos;
            float r = sqrt(pow(cam_to_obj[0],2) + pow(cam_to_obj[1], 2) + pow(cam_to_obj[2], 2));
            glm::vec3 prev_dir = direction;
            direction[0] = cos(verticalAngle)*sin(horizontalAngle);
            direction[1] = sin(verticalAngle);
            direction[2] = cos(verticalAngle)*cos(horizontalAngle);
            cam_pos = cam_pos + prev_dir *r - direction *r;
        }
    } else {
        mouseMovePressed = false;
        mouseRotatePressed = false;
        mouseZoomPressed = false;
    }

    ProjectionMatrix = glm::perspective(glm::radians(FoV), 
        1024.0f / 768.0f, 0.1f, 100.0f);
    right = glm::vec3(sin(horizontalAngle - PI / 2), 0, 
        cos(horizontalAngle - PI / 2));
    up = -glm::cross(right, direction);
    ViewMatrix = glm::lookAt(cam_pos, cam_pos + direction, up);
}

void Visualization::controlView(float deltaTime) {
    
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
        cam_pos += direction*deltaTime*speed;
    }
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        cam_pos -= direction*deltaTime*speed;
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        cam_pos += right*deltaTime*speed;
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        cam_pos -= right*deltaTime*speed;
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        cam_pos += up*deltaTime*speed;
    }
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        cam_pos -= up*deltaTime*speed;
    }
    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
        horizontalAngle += 0.01;
    }
    if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) {
        horizontalAngle -= 0.01;
    }
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS) {
        verticalAngle += 0.01;
    }
    if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS) {
        verticalAngle -= 0.01;
    }
    if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) {
        FoV += 0.01/PI*180;
    }

    ProjectionMatrix = glm::perspective(glm::radians(FoV), 
        1024.0f / 768.0f, 0.1f, 100.0f);
    direction[0] = cos(verticalAngle)*sin(horizontalAngle);
    direction[1] = sin(verticalAngle);
    direction[2] = cos(verticalAngle)*cos(horizontalAngle);
    right = glm::vec3(sin(horizontalAngle - PI / 2), 0, 
        cos(horizontalAngle - PI / 2));
    up = -glm::cross(right, direction);
    ViewMatrix = glm::lookAt(cam_pos, cam_pos + direction, up);
}

void Visualization::drawObjects(OBJ obj_type, glm::mat4 Model,
    glm::vec3 lightPos, vector<glm::vec3> &data, vector<glm::vec3> &normal,
    vector<glm::vec3> &color, vector<glm::vec3> &spccolor, int ind)
{
    glm::mat4 MVP = ProjectionMatrix * ViewMatrix * Model;
    if (obj_type == 0) {
        male->drawElements(MVP, Model, ViewMatrix, ProjectionMatrix, MatrixID, ModelMatrixID,
            ViewMatrixID, ProjectionMatrixID, LightID, lightPos);
    } else if (obj_type == 2) {
        floor->drawArrays(MVP, Model, ViewMatrix, ProjectionMatrix, MatrixID,
            ModelMatrixID, ViewMatrixID, ProjectionMatrixID, LightID, lightPos,
            data, normal, color, spccolor);
    } else if (obj_type == 3) {
        axis->drawLines(MVP, Model, ViewMatrix, ProjectionMatrix, MatrixID,
            ModelMatrixID, ViewMatrixID, ProjectionMatrixID, LightID, lightPos,
            data, color, spccolor);
    } else if (obj_type == 5) {
        sphere->drawElements(MVP, Model, ViewMatrix, ProjectionMatrix, MatrixID, ModelMatrixID,
            ViewMatrixID, ProjectionMatrixID, LightID, lightPos);
    }
}

void Visualization::cleanUp() {
	// Cleanup VBO and shader
    male->cleanUp();
    floor->cleanUp();
    axis->cleanUp();

	glDeleteProgram(programID);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();
}

void Visualization::readObj(const char * path, glm::vec3 &color, float alpha, OBJ obj) {
	// Read our .obj file
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec2> uvs;
	std::vector<glm::vec3> normals;
	bool res = loadOBJ(path, vertices, uvs, normals);

	// Load it into a VBO
	vector<unsigned short> indices;
	vector<glm::vec3> indexed_vertices;
	vector<glm::vec2> indexed_uvss;
	vector<glm::vec3> indexed_color_tmp;
	vector<glm::vec3> indexed_normals;
	indexVBO(vertices, uvs, normals, indices, indexed_vertices,
        indexed_uvss, indexed_color_tmp, indexed_normals);
    
    vector<glm::vec3> indexed_color;
    vector<glm::vec3> indexed_spccolor;

    glm::vec3 vec_alpha;
    vec_alpha.x = alpha;
    vec_alpha.y = alpha;
    vec_alpha.z = alpha;

	for (int i = 0; i < indexed_color_tmp.size(); i ++) {
        indexed_color.push_back(color);
        indexed_spccolor.push_back(vec_alpha);
	}

    if (obj == MALE) {
        for (int i = 0; i < indexed_vertices.size(); i++) {
            indexed_vertices[i][0] = indexed_vertices[i][0];
            indexed_vertices[i][1] = indexed_vertices[i][1] + 0.055;
            indexed_vertices[i][2] = indexed_vertices[i][2] - 0.01;

        }
        male = make_unique<Object>();
        male->setElement(indexed_vertices, indexed_color, indexed_normals, indices,
            indexed_spccolor);
        male->indices_size = indices.size()/3;
    } else {
        sphere = make_unique<Object>();
        sphere->setElement(indexed_vertices, indexed_color, indexed_normals, indices,
            indexed_spccolor);
        sphere->indices_size = indices.size()/3;
    }
}

void Visualization::setHexa(vector<glm::vec3>&hexa_pnts, vector<unsigned short>&indices,
    vector<glm::vec3> &hexa_color, vector<glm::vec3> &hexa_normal, vector<glm::vec3> &hexa_spccolor) {
    male = make_unique<Object>();
    male->setElement(hexa_pnts, hexa_color, hexa_normal, indices,
        hexa_spccolor);
    male->indices_size = indices.size()/3;
}

void Visualization::load_franka(float alpha) {
	std::vector<const char*> objdir_vec;
	std::vector<const char*> objrootdir_vec;

	char objdir[1000];        
	char objrootdir[100];
	char *t1 = "/home/mjlee/workspace/continuum_dynamics/cpp/data/franka/Franka_Link";
    for (int link_num = 0; link_num < 8; link_num++) {        
        std::vector<float>  positions;
        std::vector<unsigned int> indices;
        std::vector<glm::vec3> indexed_vertices;
        std::vector<glm::vec3> indexed_normals;
        int vertex_num;
        char t2[2];
        sprintf(t2, "%d", link_num);
        char *t3 = "_assy_OBJ/Franka_Link";
        char *t4 = "_assy.obj";
        sprintf(objdir, "%s%s%s%s%s", t1, t2, t3, t2, t4);
        char *t5 = "_assy_OBJ/";
        sprintf(objrootdir, "%s%s%s", t1, t2, t5);
        objdir_vec.push_back(objdir);
        objrootdir_vec.push_back(objrootdir);

        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
		std::string err;
        bool res = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
            objdir_vec[0], objrootdir_vec[0], true);

        for (size_t i = 0; i < shapes.size(); i++) {
            for (size_t ind = 0; ind < shapes[0].mesh.indices.size(); ind++) {
                indices.push_back(shapes[0].mesh.indices[ind].vertex_index);
            }
        }
        vertex_num = 0;
        for (size_t i = 0; i < shapes.size(); i++) {
            vertex_num += shapes[i].mesh.indices.size();
        }
        for (size_t v = 0; v < attrib.vertices.size()/3; v++)
        {
            glm::vec3 vertex_tmp;
            vertex_tmp.x = attrib.vertices[3 * v + 0];
            vertex_tmp.y = attrib.vertices[3 * v + 1];
            vertex_tmp.z = attrib.vertices[3 * v + 2];
            indexed_vertices.push_back(vertex_tmp);
        }
        for (size_t n = 0; n < attrib.normals.size() / 3; n++)
        {
            glm::vec3 normal_tmp;
            normal_tmp.x = attrib.normals[3 * n + 0];
            normal_tmp.y = attrib.normals[3 * n + 1];
            normal_tmp.z = attrib.normals[3 * n + 2];
            indexed_normals.push_back(normal_tmp);
        }

        std::vector<glm::vec3> indexed_color(attrib.vertices.size() / 3);
        std::vector<glm::vec3> indexed_color_a(attrib.vertices.size() / 3);
        std::vector<glm::vec3> indexed_color_s(attrib.vertices.size() / 3);
        std::vector<glm::vec3> indexed_color_alpha(attrib.vertices.size() / 3);
        glm::vec3 vec3_alpha;
        vec3_alpha.x = alpha;
        vec3_alpha.y = alpha;
        vec3_alpha.z = alpha;
        for (size_t i = 0; i < shapes.size(); i++) 
        {
            size_t index_offset = 0;
            for (size_t f = 0; f < shapes[i].mesh.num_face_vertices.size(); f++) 
            {
                size_t fnum = shapes[i].mesh.num_face_vertices[f];
                for (size_t v = 0; v < fnum; v++) {
                    tinyobj::index_t idx = shapes[i].mesh.indices[index_offset + v];
                    int material_id = shapes[i].mesh.material_ids[f];
                    glm::vec3 color_tmp;
                    glm::vec3 color_a;
                    glm::vec3 color_s;
                    color_tmp.x = materials[material_id].diffuse[0];
                    color_tmp.y = materials[material_id].diffuse[1];
                    color_tmp.z = materials[material_id].diffuse[2];
                    color_a.x = materials[material_id].ambient[0];
                    color_a.y = materials[material_id].ambient[1];
                    color_a.z = materials[material_id].ambient[2];
                    color_s.x = materials[material_id].specular[0];
                    color_s.y = materials[material_id].specular[1];
                    color_s.z = materials[material_id].specular[2];
                    indexed_color[idx.vertex_index] = color_tmp;
                    indexed_color_a[idx.vertex_index] = color_a;
                    indexed_color_s[idx.vertex_index] = color_s;
                    indexed_color_alpha[idx.vertex_index] = vec3_alpha;
                }
                index_offset += fnum;
            }
        }
        franka[link_num] = make_unique<Object>();
        franka[link_num]->setElement_franka(indexed_vertices, indexed_color, indexed_normals,
            indexed_color_a, indexed_color_s, indices, indexed_color_alpha);
        franka[link_num]->indices_size = vertex_num;
    }
}

void Visualization::load_ATI(float alpha) {
    cout << "load ATI" << endl;
    const char* objroot = "/home/mjlee/workspace/continuum_dynamics/cpp/data/FT/";
    const char* objdir ="/home/mjlee/workspace/continuum_dynamics/cpp/data/FT/Assem_1.obj";
    std::vector<float>  positions;
    std::vector<unsigned int> indices;
    std::vector<glm::vec3> indexed_vertices;
    std::vector<glm::vec3> indexed_normals;
    int vertex_num;

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;
    bool res = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
        objdir, objroot, true);

    for (size_t i = 0; i < shapes.size(); i++) {
        for (size_t ind = 0; ind < shapes[i].mesh.indices.size(); ind++) {
            indices.push_back(shapes[i].mesh.indices[ind].vertex_index);
        }
    }
    vertex_num = 0;
    for (size_t i = 0; i < shapes.size(); i++) {
        vertex_num += shapes[i].mesh.indices.size();
    }
    for (size_t v = 0; v < attrib.vertices.size()/3; v++)
    {
        glm::vec3 vertex_tmp;
        vertex_tmp.x = attrib.vertices[3 * v + 0];
        vertex_tmp.y = attrib.vertices[3 * v + 1];
        vertex_tmp.z = attrib.vertices[3 * v + 2];
        indexed_vertices.push_back(vertex_tmp);
    }
    for (size_t n = 0; n < attrib.normals.size() / 3; n++)
    {
        glm::vec3 normal_tmp;
        normal_tmp.x = attrib.normals[3 * n + 0];
        normal_tmp.y = attrib.normals[3 * n + 1];
        normal_tmp.z = attrib.normals[3 * n + 2];
        indexed_normals.push_back(normal_tmp);
    }
    std::vector<glm::vec3> indexed_color(attrib.vertices.size() / 3);
    std::vector<glm::vec3> indexed_color_a(attrib.vertices.size() / 3);
    std::vector<glm::vec3> indexed_color_s(attrib.vertices.size() / 3);
    std::vector<glm::vec3> indexed_color_alpha(attrib.vertices.size() / 3);

    glm::vec3 vec3_alpha;
    vec3_alpha.x = alpha;
    vec3_alpha.y = alpha;
    vec3_alpha.z = alpha;
    cout << "material: " << materials.size() << endl;
    for (size_t i = 0; i < shapes.size(); i++) 
    {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[i].mesh.num_face_vertices.size(); f++) 
        {
            size_t fnum = shapes[i].mesh.num_face_vertices[f];
            for (size_t v = 0; v < fnum; v++) {
                tinyobj::index_t idx = shapes[i].mesh.indices[index_offset + v];
                int material_id = shapes[i].mesh.material_ids[f];
                if (material_id == -1) {
                    material_id = 0;
                }
                glm::vec3 color_tmp;
                glm::vec3 color_a;
                glm::vec3 color_s;
                color_tmp.x = materials[material_id].diffuse[0];
                color_tmp.y = materials[material_id].diffuse[1];
                color_tmp.z = materials[material_id].diffuse[2];
                color_a.x = materials[material_id].ambient[0];
                color_a.y = materials[material_id].ambient[1];
                color_a.z = materials[material_id].ambient[2];
                color_s.x = materials[material_id].specular[0];
                color_s.y = materials[material_id].specular[1];
                color_s.z = materials[material_id].specular[2];
                indexed_color[idx.vertex_index] = color_tmp;
                indexed_color_a[idx.vertex_index] = color_a;
                indexed_color_s[idx.vertex_index] = color_s;
                indexed_color_alpha[idx.vertex_index] = vec3_alpha;
            }
            index_offset += fnum;
        }
    }
    ATI = make_unique<Object>();
    ATI->setElement_franka(indexed_vertices, indexed_color, indexed_normals,
        indexed_color_a, indexed_color_s, indices, indexed_color_alpha);
    ATI->indices_size = vertex_num;
}

void Visualization::load_ATI2(float alpha) {
    cout << "load ATI" << endl;
    const char* objroot = "/home/mjlee/workspace/continuum_dynamics/cpp/data/FT/";
    const char* objdir ="/home/mjlee/workspace/continuum_dynamics/cpp/data/FT/Assem_1.obj";
    std::vector<float>  positions;
    std::vector<unsigned int> indices;
    std::vector<glm::vec3> indexed_vertices;
    std::vector<glm::vec3> indexed_normals;
    int vertex_num;

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;
    bool res = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
        objdir, objroot, true);

    for (size_t i = 0; i < shapes.size(); i++) {
        for (size_t ind = 0; ind < shapes[i].mesh.indices.size(); ind++) {
            indices.push_back(shapes[i].mesh.indices[ind].vertex_index);
        }
    }
    vertex_num = 0;
    for (size_t i = 0; i < shapes.size(); i++) {
        vertex_num += shapes[i].mesh.indices.size();
    }
    for (size_t v = 0; v < attrib.vertices.size()/3; v++)
    {
        glm::vec3 vertex_tmp;
        vertex_tmp.x = attrib.vertices[3 * v + 0];
        vertex_tmp.y = attrib.vertices[3 * v + 1];
        vertex_tmp.z = attrib.vertices[3 * v + 2];
        indexed_vertices.push_back(vertex_tmp);
    }
    for (size_t n = 0; n < attrib.normals.size() / 3; n++)
    {
        glm::vec3 normal_tmp;
        normal_tmp.x = attrib.normals[3 * n + 0];
        normal_tmp.y = attrib.normals[3 * n + 1];
        normal_tmp.z = attrib.normals[3 * n + 2];
        indexed_normals.push_back(normal_tmp);
    }
    std::vector<glm::vec3> indexed_color(attrib.vertices.size() / 3);
    std::vector<glm::vec3> indexed_color_a(attrib.vertices.size() / 3);
    std::vector<glm::vec3> indexed_color_s(attrib.vertices.size() / 3);
    std::vector<glm::vec3> indexed_color_alpha(attrib.vertices.size() / 3);

    glm::vec3 vec3_alpha;
    vec3_alpha.x = alpha;
    vec3_alpha.y = alpha;
    vec3_alpha.z = alpha;
    cout << "material: " << materials.size() << endl;
    for (size_t i = 0; i < shapes.size(); i++) 
    {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[i].mesh.num_face_vertices.size(); f++) 
        {
            size_t fnum = shapes[i].mesh.num_face_vertices[f];
            for (size_t v = 0; v < fnum; v++) {
                tinyobj::index_t idx = shapes[i].mesh.indices[index_offset + v];
                int material_id = shapes[i].mesh.material_ids[f];
                if (material_id == -1) {
                    material_id = 0;
                }
                glm::vec3 color_tmp;
                glm::vec3 color_a;
                glm::vec3 color_s;
                color_tmp.x = materials[material_id].diffuse[0];
                color_tmp.y = materials[material_id].diffuse[1];
                color_tmp.z = materials[material_id].diffuse[2];
                color_a.x = materials[material_id].ambient[0];
                color_a.y = materials[material_id].ambient[1];
                color_a.z = materials[material_id].ambient[2];
                color_s.x = materials[material_id].specular[0];
                color_s.y = materials[material_id].specular[1];
                color_s.z = materials[material_id].specular[2];
                indexed_color[idx.vertex_index] = color_tmp;
                indexed_color_a[idx.vertex_index] = color_a;
                indexed_color_s[idx.vertex_index] = color_s;
                indexed_color_alpha[idx.vertex_index] = vec3_alpha;
            }
            index_offset += fnum;
        }
    }
    ATI2 = make_unique<Object>();
    ATI2->setElement_franka(indexed_vertices, indexed_color, indexed_normals,
        indexed_color_a, indexed_color_s, indices, indexed_color_alpha);
    ATI2->indices_size = vertex_num;
}
void Visualization::setArrays(int floor_size) {
    cout << "setarrays" << endl;
    axis = make_unique<Object>();
    axis->setBuffer(3*2);

    floor = make_unique<Object>();
    floor->setBuffer(floor_size * 3);
}