#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <cmath>
#include <utility>
#include <vector>
#include <list>
#include <map>
#include <queue>
#include <stack> 
#include "filesystem.h"
#include "shader_s.h"
//#include "Solver/Rubik.cpp"
#include "optimal_solver.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define SIN(x) sin(x * 3.141592653589/180) 
#define COS(x) cos(x * 3.141592653589/180)  

#define RED glm::vec3(1.0,0.0,0.0)
#define GREEN glm::vec3(0.0,1.0,0.0)
#define BLUE glm::vec3(0.0,0.0,1.0)
#define WHITE glm::vec3(1.0,1.0,1.0)
#define YELLOW glm::vec3(1.0,1.0,0.0)
#define ORANGE glm::vec3(1.0,0.5,0.0)

#define menora <
#define print <<
#define mayora >
#define NCUBES 27
#define limite 180





void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);

using namespace std;




unsigned int *textures[6];

glm::vec3 centerPositions[] = {
    glm::vec3(1.0f,  -1.0f,  -1.0f), };

glm::vec3 NegcenterPositions[] = {
    glm::vec3(-1.0f,  1.0f,  1.0f), };




// camera
glm::vec3 cameraPos = glm::vec3(3.0f, 1.0f, 10.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f); 
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

bool firstMouse = true;
float yaw = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float pitch = 0.0f;
float lastX = 800.0f / 2.0;
float lastY = 600.0 / 2.0;
float fov = 45.0f;

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

float mixValue = 0.0f;

bool setVBOChange = false;
vector <float> color_actual = { 1.0f, 0.0f, 0.0f };
bool vali_tri = false;
bool vali_cuad = false;
bool vali_rombo = false;

int cont_tri = 0;
int cont_rombo = 0;
int cont_estrella = 0;
int cont_cuadrado = 0;
int valid1 = false;
int valid2 =0;
int valid3 = 0;
int valicamara = 0;
int valicamara2 = 0;
int valiInicial = 0;

bool valid4 = false;
bool valiAnimacion = false;


int valisolver = 0;
int valid = 0;
float* ptr_vertices = nullptr;
float* ptr_verticesT = nullptr;
float* ptr_verticesC = nullptr;
float* ptr_verticesR = nullptr;
float* ptr_verticesE = nullptr;
int cont = 0;
int cont1 = 0;
int n_elm =192;
int n_ind = 24;
float pto_medio[2];

queue <string> solucion;

struct Vertice {
    float x, y, z, a, b, c, n, m;
    Vertice(float x_ = 0, float y_ = 0, float z_ = 0, float a_ = 0, float b_ = 0, float c_ = 0, float n_ = 0, float m_ = 0) : x(x_), y(y_), z(z_), a(a_), b(b_), c(c_), n(n_), m(m_) {}
};

//          a
//
//
//      b       c

struct Triangulo {
    Vertice a, b, c;
    Triangulo(Vertice x_, Vertice y_, Vertice z_) {
        a = x_;
        b = y_;
        c = z_;
        a.n = 1.0f; //derecha
        a.m = 0.0f;
        b.n = 0.0f; //izquierda
        b.m = 0.0f;
        c.n = 0.5f; //arriba
        c.m = 1.0f;

    }
    //int nivel;
    //char posicion;//('A','C', 'I', 'D')

};

queue <Triangulo> actuales_T;


void dibujar_Tri(Triangulo tri) {

    unsigned int indices[] = {  // note that we start from 0!
          0, 1, 2,  // first Triangle      
    };
}



void act_Vert_1_tr(Triangulo tri) {
    actuales_T.push(tri);
    float vertices_temp[] = { tri.a.x,tri.a.y,tri.a.z,tri.a.a,tri.a.b,tri.a.c,tri.a.n,tri.a.m,tri.b.x,tri.b.y,tri.b.z,tri.b.a,tri.b.b,tri.b.c,tri.b.n,tri.b.m,tri.c.x,tri.c.y,tri.c.z,tri.c.a,tri.c.b,tri.c.c,tri.c.n,tri.c.m };
    unsigned int n = n_elm;
    unsigned int n1 = n_ind;

    int c = n + 24;
    float* p1 = new float[c];


    n_elm += 24;
    n_ind += 3;


    for (int i = 0; i < n; i++) {
        p1[i] = ptr_vertices[i];
    }



    for (int i = 0; i < 24; i++) {
        p1[n + i] = vertices_temp[i];
    }

    delete[] ptr_vertices;

    ptr_vertices = &(p1[0]);



}





//int time = 0;
int timebase = 0;
GLfloat angulo = 5;
GLfloat origen = -8;
GLfloat direccion = -1;






std::string& rtrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
    str.erase(str.find_last_not_of(chars) + 1);
    return str;
}




typedef pair<int, int> par_mapa;

unsigned int VBO[NCUBES], VAO[NCUBES];
float* punteros[NCUBES];

map <int, int> indice_tag;



float toRadians(int x) {return x * 3.141592653589 / 180.0;}

int valiLeft = 0;
int valiFront = 0;
int valiUp = 0;
int valiRight = 0;
int valiDown = 0;
int valiBack = 0;






glm::mat4 rotarRight(int& valiRight, int i, glm::mat4 model) {
    vector<int> my_list = { 0,3,6,9,12,15,18,21,24 };
    vector<int> cambio = { 18,9,0,21,12,3,24,15,6 };
    map<int, int>::iterator q = indice_tag.find(i);
    //cout print "entra" print i print endl;

    if (find(my_list.begin(), my_list.end(), i) != my_list.end()) {

        //cout print "compara";
        model = glm::translate(model, centerPositions[0]);
        model = glm::rotate(model, toRadians(5 * valiRight), glm::vec3(1.0f, 0.0f, 0.0f));
        model = glm::translate(model, NegcenterPositions[0]);

        if (valiAnimacion == true) {

            if (i == my_list[0] || i == my_list[1] || i == my_list[2]) {
                model = glm::translate(model, glm::vec3(0.0f, 0.0f, 1.0f));
            }

            if (i == my_list[0] || i == my_list[3] || i == my_list[6]) {
                model = glm::translate(model, glm::vec3(0.0f, 1.0f, 0.0f));
            }


            if (i == my_list[2] || i == my_list[5] || i == my_list[8]) {
                model = glm::translate(model, glm::vec3(0.0f, -1.0f, 0.0f));
            }


            if (i == my_list[6] || i == my_list[7] || i == my_list[8]) {
                model = glm::translate(model, glm::vec3(0.0f, 0.0f, -1.0f));
            }
        }
        if (valiRight == 17) {
            //cout print "actualizamos cubo" print i print endl;
            glm::quat MyQuaternion;
            // Direct specification of the 4 components
            // You almost never use this directly
            // RotationAngle is in radians
            glm::vec3 EulerAngles(1.5708, 0, 0);
            MyQuaternion = glm::quat(EulerAngles);
            for (int s = 0; s < 36; ++s) {
                //cout print "antes:" print i print endl;
                //cout print "x=" print  punteros[i][s * 8] print " ,y=" print punteros[i][(s * 8) + 1] print " ,z=" print punteros[i][(s * 8) + 2] print endl;
                glm::vec3 point = { punteros[q->second][s * 8], punteros[q->second][(s * 8) + 1], punteros[q->second][(s * 8) + 2] };
                glm::vec3 rotated_point = centerPositions[0] + (MyQuaternion * (point - centerPositions[0]));
                punteros[q->second][s * 8] = rotated_point[0];
                punteros[q->second][(s * 8) + 1] = rotated_point[1];
                punteros[q->second][(s * 8) + 2] = rotated_point[2];
                //cout print "despues:" print i print endl;
                //cout print "x=" print  punteros[i][s * 8] print " ,y=" print punteros[i][(s * 8) + 1] print " ,z=" print punteros[i][(s * 8) + 2] print endl;
            }
        }
    }
    if (i == 26) {
        valiRight += 1;
        if (valiRight == 18) {
            //cout print "actualizar" print endl;

            map<int, int>::iterator p;

            vector<float > temp;
            for (int t = 0; t menora 9; ++t) {
                p = indice_tag.find(cambio[t]);
                temp.push_back(p->second);
            }
            //cout print "mapa" print endl;
            for (int t = 0; t menora 9; ++t) {
                p = indice_tag.find(my_list[t]);
                //cout print p->first print " " print p->second print endl;
                p->second = temp[t];
                //cout print p->first print " " print p->second print endl;


            }

            valiRight = 0;
        }
    }


    //cout print valiLeft print endl;
    return model;
}


glm::mat4 rotarLeft(int &valiLeft, int i, glm::mat4 model) {
    vector<int> my_list= { 2,5,8,11,14,17,20,23,26 };
    vector<int> cambio = { 20,11,2,23,14,5,26,17,8 };
    map<int, int>::iterator q= indice_tag.find(i);
   
    if (find(my_list.begin(), my_list.end(),i) != my_list.end()) {
        model = glm::translate(model, centerPositions[0]);
        model = glm::rotate(model, toRadians(5 * valiLeft), glm::vec3(1.0f, 0.0f, 0.0f));
        model = glm::translate(model, NegcenterPositions[0]);

        if (valiAnimacion == true) {
            if (i == 2 || i == 5 || i == 8) {
                model = glm::translate(model, glm::vec3(0.0f, 0.0f, 1.0f));
            }

            if (i == 2 || i == 11 || i == 20) {
                model = glm::translate(model, glm::vec3(0.0f, 1.0f, 0.0f));
            }

            if (i == 8 || i == 17 || i == 26) {
                model = glm::translate(model, glm::vec3(0.0f, -1.0f, 0.0f));
            }


            if (i == 20 || i == 23 || i == 26) {
                model = glm::translate(model, glm::vec3(0.0f, 0.0f, -1.0f));
            }
        }
        if (valiLeft == 17) {
            //cout print "actualizamos cubo" print i print endl;
            glm::quat MyQuaternion;
            // Direct specification of the 4 components
            // You almost never use this directly
            // RotationAngle is in radians
            glm::vec3 EulerAngles(1.5708, 0, 0);
            MyQuaternion = glm::quat(EulerAngles);
            for (int s = 0; s < 36; ++s) {                
                //cout print "antes:" print i print endl;
                //cout print "x=" print  punteros[i][s * 8] print " ,y=" print punteros[i][(s * 8) + 1] print " ,z=" print punteros[i][(s * 8) + 2] print endl;
                glm::vec3 point = { punteros[q->second][s * 8], punteros[q->second][(s * 8) + 1], punteros[q->second][(s * 8) + 2] };
                glm::vec3 rotated_point = centerPositions[0] + (MyQuaternion * (point - centerPositions[0]));
                punteros[q->second][s * 8] = rotated_point[0];
                punteros[q->second][(s * 8) + 1] = rotated_point[1];
                punteros[q->second][(s * 8) + 2] = rotated_point[2];
                //cout print "despues:" print i print endl;
                //cout print "x=" print  punteros[i][s * 8] print " ,y=" print punteros[i][(s * 8) + 1] print " ,z=" print punteros[i][(s * 8) + 2] print endl;
            }
        }    

    }
    if (i == 26) {
        valiLeft += 1;
        if (valiLeft == 18) {
            map<int, int>::iterator p;
            vector<float > temp;
            for (int t = 0; t menora 9; ++t) {
                p = indice_tag.find(cambio[t]);
                temp.push_back(p->second);
            }
            //cout print "mapa" print endl;
            for (int t = 0; t menora 9; ++t) {
                p = indice_tag.find(my_list[t]);
                p->second = temp[t];
            }
            valiLeft = 0;
        }
    }

    
    //cout print valiLeft print endl;
    return model;
}



glm::mat4 rotarDown(int& valiDown, int i, glm::mat4 model) {
    vector<int> my_list = { 6,7,8,15,16,17,24,25,26 };
    //vector<int> mylist = { 18,19,20,21,22,23,24,25,26 };
    //vector<int> cambio = { 20,23,26,19,22,25,18,21,24 };
    vector<int> cambio = {24,15,6,25,16,7,26,17,8 };
    map<int, int>::iterator q = indice_tag.find(i);
    //cout print "entra" print i print endl;

    if (find(my_list.begin(), my_list.end(), i) != my_list.end()) {

        //cout print "compara";
        model = glm::translate(model, centerPositions[0]);
        model = glm::rotate(model, toRadians(5 * valiDown), glm::vec3(0.0f, 1.0f, 0.0f));
        model = glm::translate(model, NegcenterPositions[0]);

        if (valiAnimacion == true) {
            if (i == my_list[0] || i == my_list[1] || i == my_list[2]) {
                model = glm::translate(model, glm::vec3(0.0f, 0.0f, 1.0f));
            }

            if (i == my_list[0] || i == my_list[3] || i == my_list[6]) {
                model = glm::translate(model, glm::vec3(-1.0f, 0.0f, 0.0f));
            }


            if (i == my_list[2] || i == my_list[5] || i == my_list[8]) {
                model = glm::translate(model, glm::vec3(1.0f, 0.0f, 0.0f));
            }


            if (i == my_list[6] || i == my_list[7] || i == my_list[8]) {
                model = glm::translate(model, glm::vec3(0.0f, 0.0f, -1.0f));
            }
        }



        if (valiDown == 17) {
            //cout print "actualizamos cubo" print i print endl;
            glm::quat MyQuaternion;
            // Direct specification of the 4 components
            // You almost never use this directly
            // RotationAngle is in radians
            glm::vec3 EulerAngles(0, 1.5708,0);
            MyQuaternion = glm::quat(EulerAngles);
            for (int s = 0; s < 36; ++s) {
                //cout print "antes:" print i print endl;
                //cout print "x=" print  punteros[i][s * 8] print " ,y=" print punteros[i][(s * 8) + 1] print " ,z=" print punteros[i][(s * 8) + 2] print endl;
                glm::vec3 point = { punteros[q->second][s * 8], punteros[q->second][(s * 8) + 1], punteros[q->second][(s * 8) + 2] };
                glm::vec3 rotated_point = centerPositions[0] + (MyQuaternion * (point - centerPositions[0]));
                punteros[q->second][s * 8] = rotated_point[0];
                punteros[q->second][(s * 8) + 1] = rotated_point[1];
                punteros[q->second][(s * 8) + 2] = rotated_point[2];
                //cout print "despues:" print i print endl;
                //cout print "x=" print  punteros[i][s * 8] print " ,y=" print punteros[i][(s * 8) + 1] print " ,z=" print punteros[i][(s * 8) + 2] print endl;
            }
        }
    }
    if (i == 26) {
        valiDown += 1;
        if (valiDown == 18) {
            //cout print "actualizar" print endl;

            map<int, int>::iterator p;

            vector<float > temp;
            for (int t = 0; t menora 9; ++t) {
                p = indice_tag.find(cambio[t]);
                temp.push_back(p->second);
            }
            //cout print "mapa" print endl;
            for (int t = 0; t menora 9; ++t) {
                p = indice_tag.find(my_list[t]);
               // cout print p->first print " " print p->second print endl;
                p->second = temp[t];
               // cout print p->first print " " print p->second print endl;


            }

            valiDown = 0;
        }
    }


    //cout print valiLeft print endl;
    return model;
}



glm::mat4 rotarBack(int& valiBack, int i, glm::mat4 model) {
    vector<int> my_list = { 18,19,20,21,22,23,24,25,26 };
    vector<int> cambio = { 20,23,26,19,22,25,18,21,24 };
    map<int, int>::iterator q = indice_tag.find(i);
    //cout print "entra" print i print endl;

    if (find(my_list.begin(), my_list.end(), i) != my_list.end()) {

        //cout print "compara";
        model = glm::translate(model, centerPositions[0]);
        model = glm::rotate(model, toRadians(5 * valiBack), glm::vec3(0.0f, 0.0f, 1.0f));
        model = glm::translate(model, NegcenterPositions[0]);
        if( valiAnimacion==true){
            if (i == my_list[0] || i == my_list[1] || i == my_list[2]) {
                model = glm::translate(model, glm::vec3(0.0f, 1.0f, 0.0f));
            }

            if (i == my_list[0] || i == my_list[3] || i == my_list[6]) {
                model = glm::translate(model, glm::vec3(-1.0f, 0.0f, 0.0f));
            }


            if (i == my_list[2] || i == my_list[5] || i == my_list[8]) {
                model = glm::translate(model, glm::vec3(1.0f, 0.0f, 0.0f));
            }


            if (i == my_list[6] || i == my_list[7] || i == my_list[8]) {
                model = glm::translate(model, glm::vec3(0.0f, -1.0f, 0.0f));
            }
        }
        if (valiBack == 17) {
            //cout print "actualizamos cubo" print i print endl;
            glm::quat MyQuaternion;
            // Direct specification of the 4 components
            // You almost never use this directly
            // RotationAngle is in radians
            glm::vec3 EulerAngles(0, 0, 1.5708);
            MyQuaternion = glm::quat(EulerAngles);
            for (int s = 0; s < 36; ++s) {
                //cout print "antes:" print i print endl;
                //cout print "x=" print  punteros[i][s * 8] print " ,y=" print punteros[i][(s * 8) + 1] print " ,z=" print punteros[i][(s * 8) + 2] print endl;
                glm::vec3 point = { punteros[q->second][s * 8], punteros[q->second][(s * 8) + 1], punteros[q->second][(s * 8) + 2] };
                glm::vec3 rotated_point = centerPositions[0] + (MyQuaternion * (point - centerPositions[0]));
                punteros[q->second][s * 8] = rotated_point[0];
                punteros[q->second][(s * 8) + 1] = rotated_point[1];
                punteros[q->second][(s * 8) + 2] = rotated_point[2];
                //cout print "despues:" print i print endl;
                //cout print "x=" print  punteros[i][s * 8] print " ,y=" print punteros[i][(s * 8) + 1] print " ,z=" print punteros[i][(s * 8) + 2] print endl;
            }
        }
    }
    if (i == 26) {
        valiBack += 1;
        if (valiBack == 18) {
            //cout print "actualizar" print endl;

            map<int, int>::iterator p;

            vector<float > temp;
            for (int t = 0; t menora 9; ++t) {
                p = indice_tag.find(cambio[t]);
                temp.push_back(p->second);
            }
            //cout print "mapa" print endl;
            for (int t = 0; t menora 9; ++t) {
                p = indice_tag.find(my_list[t]);
                //cout print p->first print " " print p->second print endl;
                p->second = temp[t];
                //cout print p->first print " " print p->second print endl;


            }

            valiBack = 0;
        }
    }


    //cout print valiLeft print endl;
    return model;
}


glm::mat4 rotarFront(int& valiFront, int i, glm::mat4 model) {
    vector<int> my_list = { 0,1,2,3,4,5,6,7,8};
    vector<int> cambio = { 2,5,8,1,4,7,0,3,6};
    map<int, int>::iterator q = indice_tag.find(i);
    //cout print "entra" print i print endl;

    if (find(my_list.begin(), my_list.end(),i) != my_list.end()) {

        //cout print "compara";
        model = glm::translate(model, centerPositions[0]);
        model = glm::rotate(model, toRadians(5 * valiFront), glm::vec3(0.0f, 0.0f, 1.0f));
        model = glm::translate(model, NegcenterPositions[0]);

        if (valiAnimacion == true) {

            if (i == my_list[0] || i == my_list[1] || i == my_list[2]) {
                model = glm::translate(model, glm::vec3(0.0f, 1.0f, 0.0f));
            }

            if (i == my_list[0] || i == my_list[3] || i == my_list[6]) {
                model = glm::translate(model, glm::vec3(-1.0f, 0.0f, 0.0f));
            }


            if (i == my_list[2] || i == my_list[5] || i == my_list[8]) {
                model = glm::translate(model, glm::vec3(1.0f, 0.0f, 0.0f));
            }


            if (i == my_list[6] || i == my_list[7] || i == my_list[8]) {
                model = glm::translate(model, glm::vec3(0.0f, -1.0f, 0.0f));
            }
        }
        if (valiFront == 17) {
            //cout print "actualizamos cubo" print i print endl;
            glm::quat MyQuaternion;
            // Direct specification of the 4 components
            // You almost never use this directly
            // RotationAngle is in radians
            glm::vec3 EulerAngles(0, 0, 1.5708);
            MyQuaternion = glm::quat(EulerAngles);
            for (int s = 0; s < 36; ++s) {
                //cout print "antes:" print i print endl;
                //cout print "x=" print  punteros[i][s * 8] print " ,y=" print punteros[i][(s * 8) + 1] print " ,z=" print punteros[i][(s * 8) + 2] print endl;
                glm::vec3 point = { punteros[q->second][s * 8], punteros[q->second][(s * 8) + 1], punteros[q->second][(s * 8) + 2] };
                glm::vec3 rotated_point = centerPositions[0] + (MyQuaternion * (point - centerPositions[0]));
                punteros[q->second][s * 8] = rotated_point[0];
                punteros[q->second][(s * 8) + 1] = rotated_point[1];
                punteros[q->second][(s * 8) + 2] = rotated_point[2];
                //cout print "despues:" print i print endl;
                //cout print "x=" print  punteros[i][s * 8] print " ,y=" print punteros[i][(s * 8) + 1] print " ,z=" print punteros[i][(s * 8) + 2] print endl;
            }
        }
    }
    if (i == 26) {
        valiFront += 1;
        if (valiFront == 18) {
            //cout print "actualizar" print endl;

            map<int, int>::iterator p;

            vector<float > temp;
            for (int t = 0; t menora 9; ++t) {
                p = indice_tag.find(cambio[t]);
                temp.push_back(p->second);
            }
            //cout print "mapa" print endl;
            for (int t = 0; t menora 9; ++t) {
                p = indice_tag.find(my_list[t]);
                //cout print p->first print " " print p->second print endl;
                p->second = temp[t];
                //cout print p->first print " " print p->second print endl;


            }

            valiFront = 0;
        }
    }


    //cout print valiLeft print endl;
    return model;
}



glm::mat4 rotarUp(int& valiUp, int i, glm::mat4 model) {
    vector<int> my_list = { 0,1,2,9,10,11,18,19,20 };
    vector<int> cambio = { 18,9,0,19,10,1,20,11,2 };
    map<int, int>::iterator q = indice_tag.find(i);
    //cout print "entra" print i print endl;

    if (find(my_list.begin(), my_list.end(), i) != my_list.end()) {

        //cout print "compara";
        model = glm::translate(model, centerPositions[0]);
        model = glm::rotate(model, toRadians(5 * valiUp), glm::vec3(0.0f, 1.0f, 0.0f));
        model = glm::translate(model, NegcenterPositions[0]);

        if (valiAnimacion == true) {
            if (i == my_list[0] || i == my_list[1] || i == my_list[2]) {
                model = glm::translate(model, glm::vec3(0.0f, 0.0f, 1.0f));
            }

            if (i == my_list[0] || i == my_list[3] || i == my_list[6]) {
                model = glm::translate(model, glm::vec3(-1.0f, 0.0f, 0.0f));
            }


            if (i == my_list[2] || i == my_list[5] || i == my_list[8]) {
                model = glm::translate(model, glm::vec3(1.0f, 0.0f, 0.0f));
            }


            if (i == my_list[6] || i == my_list[7] || i == my_list[8]) {
                model = glm::translate(model, glm::vec3(0.0f, 0.0f, -1.0f));
            }
        }

        if (valiUp == 17) {
            //cout print "actualizamos cubo" print i print endl;
            glm::quat MyQuaternion;
            // Direct specification of the 4 components
            // You almost never use this directly
            // RotationAngle is in radians
            glm::vec3 EulerAngles(0, 1.5708, 0);
            MyQuaternion = glm::quat(EulerAngles);
            for (int s = 0; s < 36; ++s) {
                //cout print "antes:" print i print endl;
                //cout print "x=" print  punteros[i][s * 8] print " ,y=" print punteros[i][(s * 8) + 1] print " ,z=" print punteros[i][(s * 8) + 2] print endl;
                glm::vec3 point = { punteros[q->second][s * 8], punteros[q->second][(s * 8) + 1], punteros[q->second][(s * 8) + 2] };
                glm::vec3 rotated_point = centerPositions[0] + (MyQuaternion * (point - centerPositions[0]));
                punteros[q->second][s * 8] = rotated_point[0];
                punteros[q->second][(s * 8) + 1] = rotated_point[1];
                punteros[q->second][(s * 8) + 2] = rotated_point[2];
                //cout print "despues:" print i print endl;
                //cout print "x=" print  punteros[i][s * 8] print " ,y=" print punteros[i][(s * 8) + 1] print " ,z=" print punteros[i][(s * 8) + 2] print endl;
            }
        }
    }
    if (i == 26) {
        valiUp += 1;
        if (valiUp == 18) {
            //cout print "actualizar" print endl;

            map<int, int>::iterator p;

            vector<float > temp;
            for (int t = 0; t menora 9; ++t) {
                p = indice_tag.find(cambio[t]);
                temp.push_back(p->second);
            }
            //cout print "mapa" print endl;
            for (int t = 0; t menora 9; ++t) {
                p = indice_tag.find(my_list[t]);
                //cout print p->first print " " print p->second print endl;
                p->second = temp[t];
                //cout print p->first print " " print p->second print endl;


            }

            valiUp = 0;
        }
    }


    //cout print valiLeft print endl;
    return model;
}


void resolver(string mov) {
    if (mov == "F") {

    }

}

queue<string> moves_(string solu) {
        vector<string> temp;
       queue <string> mystack;
       std::istringstream isstream(solu);
       std::string palabra;
       while (isstream >> palabra) {
           temp.push_back(palabra);
       }
       for (int i = 0; i < temp.size(); ++i) {
           //cout print "temp " print temp[i] print endl;;
       }
       for (int i = 0; i < temp.size(); ++i) {
           char inicial=temp[i][0];
           string ini (1,temp[i][0]);
          // cout print"inicial " print inicial print endl;
           if (temp[i][1]) {
               //cout print"segundo " print temp[i][1] print endl;
               string tempo = "a'";
               char a = tempo[1];
               if (temp[i][1] == '2') {
                   string pushear = ini + "'";
                   for (int i = 0; i < 2; ++i) {
                       mystack.push(pushear);
                   }

               }              
               if (temp[i][1] == a) {
                   //cout print"apostrofe " print a print endl;
                   string pushear = ini + "'";
                  // cout print"pushear " print pushear print endl;
                   mystack.push(pushear);
               }
           }
           else {
               string pushear = ini + "'";
               for (int i = 0; i < 3; ++i) {
                   mystack.push(pushear);
               }
           }

       }

       return mystack;
}




string movimientos = "";
int main()
{


   // string a="hola mi nombre es este       ";
    //rtrim(a);
    //cout print a print "seee" print endl;
    std::cout << "** Main **" << std::endl;

    

    std::cout << "** Movimientos hechos: **" << std::endl;

    //movimientos += "R' U";

    //OptimalSolver os(6);
    //os.solve(movimientos);


/*
    std::cout << "** A-perm **" << std::endl;
    os.solve("R' F R' B2 R F' R' B2 R2");

    std::cout << "** T-perm **" << std::endl;
    os.solve("R U R' U' R' F R2 U' R' U' R U R' F'");

    std::cout << "** Y-perm **" << std::endl;
    os.solve("F R U' R' U' R U R' F' R U R' U' R' F R F'");

    std::cout << "** E-perm **" << std::endl;
    os.solve("R' U L' D2 L U' R L' U R' D2 R U' L");

    std::cout << "** Z-perm **" << std::endl;
    os.solve("R U R' U R' U' R' U R U' R' U' R2 U R U2");

    std::cout << "** Pi-COLL **" << std::endl;
    os.solve("U2 F U R U' R' U R U2 R' U' R U R' F'");

    std::cout << "** Random (15 moves) **" << std::endl;
    os.solve("U' B D R2 F' L2 R U' B' D' L B2 F' L2 R'");

    std::cout << "** Random (16 moves) **" << std::endl;
    os.solve("R' L2 D' R' L B2 L2 F D' U F D' L D2 R2 D");*/


    /*
    Cube myCube(false);
    int scrambleNum = 0;
    string a="R' F U' D2 R F R' L2 F' B2 U' F2 B' R2 B D2 B' U2 R2";
    while (std::getline(std::cin, a)) {
        ++scrambleNum;
        std::string scramble = format(a);
        std::cout << "Scramble #" << scrambleNum << ": ";
        myCube.moves(scramble);
        Cross::solveCross(myCube);
        std::cout << "Cross solved" << std::endl;
        Corners::solveCorners(myCube);
        std::cout << "Corners solved" << std::endl;
        Edges::solveEdges(myCube);
        std::cout << "Edges solved" << std::endl;;
        OLL::solveOLL(myCube);
        std::cout << "OLL solved" << std::endl;
        PLL::solvePLL(myCube);
        std::cout << "PLL solved" << std::endl;
    }
    */

    for (int j = 0; j < 6; ++j) {
        textures[j] = new unsigned int[9];
    }







    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetKeyCallback(window, key_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);


    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);


    // build and compile our shader zprogram
    // ------------------------------------
    Shader ourShader("C://Users//USER//Desktop//Universidad//actual//grafica_//GLFW_GLAD_GLUT_GLEW_cmake_project//src//TestCube1//Shaders//3.3.shader.vs", "C://Users//USER//Desktop//Universidad//actual//grafica_//GLFW_GLAD_GLUT_GLEW_cmake_project//src//TestCube1//Shaders//3.3.shader.fs");

    // set up vertex data (and buffer(s)) and configure vertex attributes
    // ------------------------------------------------------------------

    for (int i = 0; i menora NCUBES; ++i) {
        indice_tag.insert(par_mapa(i, i));
    }




    float vertices[288] = {
        // positions          // colors           // texture coords ////// cubo
           -0.5f, -0.5f, -0.5f,   1.0f, 0.0f, 0.0f,   0.0f, 0.0f,
          0.5f, -0.5f, -0.5f,     1.0f, 0.0f, 0.0f,   1.0f, 0.0f,
            0.5f,  0.5f, -0.5f,   1.0f, 0.0f, 0.0f,   1.0f, 1.0f,     ///////BACK
             0.5f,  0.5f, -0.5f,  1.0f, 0.0f, 0.0f,   1.0f, 1.0f,
            -0.5f,  0.5f, -0.5f,  1.0f, 0.0f, 0.0f,   0.0f, 1.0f,
            -0.5f, -0.5f, -0.5f,  1.0f, 0.0f, 0.0f,   0.0f, 0.0f,

             -0.5f, -0.5f, 0.5f,   0.0f, 0.9f, 0.0f,   0.0f, 0.0f,
             0.5f, -0.5f, 0.5f,    0.0f, 0.9f, 0.0f,   1.0f, 0.0f,
             0.5f,  0.5f, 0.5f,    0.0f, 0.9f, 0.0f,   1.0f, 1.0f,       //FRONT
             0.5f,  0.5f, 0.5f,    0.0f, 0.9f, 0.0f,   1.0f, 1.0f,
            -0.5f,  0.5f, 0.5f,    0.0f, 0.9f, 0.0f,   0.0f, 1.0f,
            -0.5f, -0.5f, 0.5f,    0.0f, 0.9f, 0.0f,   0.0f, 0.0f,

              -0.5f, 0.5f, 0.5f,  1.0,0.5,0.0,   0.0f, 0.0f,
          -0.5f, 0.5f, -0.5f, 1.0,0.5,0.0,  1.0f, 0.0f,
            -0.5f,  -0.5f, -0.5f, 1.0,0.5,0.0,   1.0f, 1.0f,     //left 
             -0.5f,  -0.5f, -0.5f, 1.0,0.5,0.0,  1.0f, 1.0f,
            -0.5f,  -0.5f, 0.5f, 1.0,0.5,0.0,   0.0f, 1.0f,
            -0.5f, 0.5f, 0.5f,  1.0,0.5,0.0,   0.0f, 0.0f,

              0.5f, 0.5f, 0.5f,  1.0f, 1.0f, 0.0f,   0.0f, 0.0f,
          0.5f, 0.5f, -0.5f,   1.0f, 1.0f, 0.0f,   1.0f, 0.0f,
            0.5f,  -0.5f, -0.5f, 1.0f, 1.0f, 0.0f,   1.0f, 1.0f,
             0.5f,  -0.5f, -0.5f, 1.0f, 1.0f, 0.0f,    1.0f, 1.0f,    //right
            0.5f,  -0.5f, 0.5f,   1.0f, 1.0f, 0.0f,    0.0f, 1.0f,
            0.5f, 0.5f, 0.5f,   1.0f, 1.0f, 0.0f,   0.0f, 0.0f,

              -0.5f, -0.5f, -0.5f,  1.0f, 0.8f, 0.9f,   0.0f, 0.0f,
          0.5f, -0.5f, -0.5f,   1.0f, 0.8f, 0.9f,  1.0f, 0.0f,
            0.5f,  -0.5f, 0.5f,  1.0f, 0.8f, 0.9f,  1.0f, 1.0f,
             0.5f,  -0.5f, 0.5f,   1.0f, 0.8f, 0.9f,   1.0f, 1.0f,    //down
            -0.5f,  -0.5f, 0.5f,   1.0f, 0.8f, 0.9f,   0.0f, 1.0f,
            -0.5f, -0.5f, -0.5f,   1.0f, 0.8f, 0.9f,   0.0f, 0.0f,

              -0.5f, 0.5f, -0.5f,  0.0f, 0.2f, 0.9f,   0.0f, 0.0f,
          0.5f, 0.5f, -0.5f,   0.0f, 0.2f, 0.9f,  1.0f, 0.0f,
            0.5f,  0.5f, 0.5f,  0.0f, 0.2f, 0.9f,   1.0f, 1.0f,      //TOP
             0.5f,  0.5f, 0.5f,   0.0f, 0.2f, 0.9f,   1.0f, 1.0f,
            -0.5f,  0.5f, 0.5f,   0.0f, 0.2f, 0.9f,  0.0f, 1.0f,
            -0.5f, 0.5f, -0.5f,   0.0f, 0.2f, 0.9f,   0.0f, 0.0f,

    };

    for (int j = 0; j < NCUBES; ++j) {
        punteros[j] = new float[288];
    }


    for (int i = 0; i < NCUBES; ++i) {
        for (int j = 0; j < 288; ++j) {
            punteros[i][j] = vertices[j];
        }
    }



    glm::vec3 pPositions[] = {
        glm::vec3(0.0f,  5.0f,  -5.0f), };

    glm::vec3 dPositions[] = {
       glm::vec3(0.0f,  5.0f,  0.0f), };

    pto_medio[0] = 0;

    pto_medio[1] = 0;
    float* p1 = new float[432];



    float* pt = new float[288];



    float* pc = new float[144];//432
    float* pr = new float[288];



    for (int i = 0; i < 288; i++) {
        pt[i] = vertices[i];
    }

    for (int i = 0; i < 288; i++) {
        pr[i] = vertices[i];
    }

    ptr_vertices = &(p1[0]);



    ptr_verticesT = &(pt[0]);
    ptr_verticesC = &(pc[0]);
    ptr_verticesR = &(pr[0]);



    float vec_dir[3];
    vec_dir[0] = 1.0f;
    vec_dir[1] = 0.0f;
    vec_dir[2] = 0.0f;



    //////////////////////////////////////////////////////////SE MODIFICA

    for (int j = 1; j < 3; ++j) {
        for (int i = 0; i < 36; ++i) {
            punteros[j][i * 8] = punteros[j - 1][i * 8] + vec_dir[0];
            punteros[j][(i * 8) + 1] = punteros[j - 1][(i * 8) + 1] + vec_dir[1];
            punteros[j][(i * 8) + 2] = punteros[j - 1][(i * 8) + 2] + vec_dir[2];
        }
    }


    for (int j = 3; j < 9; ++j) {
        for (int i = 0; i < 36; ++i) {
            punteros[j][i * 8] = punteros[j - 3][i * 8] + vec_dir[1];
            punteros[j][(i * 8) + 1] = punteros[j - 3][(i * 8) + 1] - vec_dir[0];
            punteros[j][(i * 8) + 2] = punteros[j - 3][(i * 8) + 2] + vec_dir[2];
        }
    }



    for (int j = 9; j < 27; ++j) {
        for (int i = 0; i < 36; ++i) {
            punteros[j][i * 8] = punteros[j - 9][i * 8] + vec_dir[1];
            punteros[j][(i * 8) + 1] = punteros[j - 9][(i * 8) + 1] - vec_dir[2];
            punteros[j][(i * 8) + 2] = punteros[j - 9][(i * 8) + 2] - vec_dir[0];
        }
    }




    //////////////////////////////////////////////////////////SE MODIFICA
    //////////////////////////////////////////////////////////SE MODIFICA
    //////////////////////////////////////////////////////////SE MODIFICA
    //////////////////////////////////////////////////////////SE MODIFICA

    for (int j = 0; j < 27; j++) {
        if ((j % 3) != 2) {
            for (int i = 18; i < 24; i++) {
                punteros[j][(i * 8) + 4] = 0.0f;
                punteros[j][(i * 8) + 5] = 0.0f;
                punteros[j][(i * 8) + 6] = 0.0f;
            }
        }
    }

    for (int j = 0; j < 27; j++) {
        if ((j % 3) != 0) {
            for (int i = 12; i < 18; i++) {
                punteros[j][(i * 8) + 4] = 0.0f;
                punteros[j][(i * 8) + 5] = 0.0f;
                punteros[j][(i * 8) + 6] = 0.0f;
            }
        }
    }

    for (int j = 0; j < 6; j++) {
        for (int i = 24; i < 30; i++) {
            punteros[j][(i * 8) + 4] = 0.0f;
            punteros[j][(i * 8) + 5] = 0.0f;
            punteros[j][(i * 8) + 6] = 0.0f;

            punteros[j + 9][(i * 8) + 4] = 0.0f;
            punteros[j + 9][(i * 8) + 5] = 0.0f;
            punteros[j + 9][(i * 8) + 6] = 0.0f;

            punteros[j + 18][(i * 8) + 4] = 0.0f;
            punteros[j + 18][(i * 8) + 5] = 0.0f;
            punteros[j + 18][(i * 8) + 6] = 0.0f;
        }
    }

    for (int j = 3; j < 9; j++) {
        for (int i = 30; i < 36; i++) {
            punteros[j][(i * 8) + 4] = 0.0f;
            punteros[j][(i * 8) + 5] = 0.0f;
            punteros[j][(i * 8) + 6] = 0.0f;

            punteros[j + 9][(i * 8) + 4] = 0.0f;
            punteros[j + 9][(i * 8) + 5] = 0.0f;
            punteros[j + 9][(i * 8) + 6] = 0.0f;

            punteros[j + 18][(i * 8) + 4] = 0.0f;
            punteros[j + 18][(i * 8) + 5] = 0.0f;
            punteros[j + 18][(i * 8) + 6] = 0.0f;
        }
    }

    for (int j = 0; j < 18; j++) {
        for (int i = 0; i < 6; i++) {
            punteros[j][(i * 8) + 4] = 0.0f;
            punteros[j][(i * 8) + 5] = 0.0f;
            punteros[j][(i * 8) + 6] = 0.0f;

        }
    }

    for (int j = 9; j < 27; j++) {
        for (int i = 6; i < 12; i++) {
            punteros[j][(i * 8) + 4] = 0.0f;
            punteros[j][(i * 8) + 5] = 0.0f;
            punteros[j][(i * 8) + 6] = 0.0f;

        }
    }





    // Vertice a(-1.0f, -0.8f, 0.0f, 1.0f, 0.0f, 0.1f, 0.0f, 0.0f);
     //Vertice b(1.0f, -0.8f, 0.0f, 0.0f, 1.0f, 0.1f, 1.0f, 0.0f);
     //Vertice c(0.0f, 0.93f, 0.0f, 0.0f, 0.0f, 1.0f, 0.5f, 1.0f);
     //Triangulo trian(a, b, c);


     //actuales_T.push(trian);



    glGenVertexArrays(NCUBES, VAO);
    glGenBuffers(NCUBES, VBO);

    for (int j = 0; j < NCUBES; ++j) {
        glBindVertexArray(VAO[j]);
        glBindBuffer(GL_ARRAY_BUFFER, VBO[j]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(punteros[j][0]) * 288, punteros[j], GL_STATIC_DRAW);


        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        // color attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        // texture coord attribute
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(2);


    }


    




    // load and create a texture 
    // -------------------------

    // cara front
    // ---------

    stbi_set_flip_vertically_on_load(true);
    for (int j = 0; j < 9; ++j) {
        string temp = "F";
        glGenTextures(1, &textures[0][j]);
        glBindTexture(GL_TEXTURE_2D, textures[0][j]);
        // set the texture wrapping parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        // set texture filtering parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // load image, create texture and generate mipmaps
        int width, height, nrChannels;
        temp += to_string(j);
        temp += ".jpg";
       // cout print temp print(0 * 9) + j print  endl;

        // tell stb_image.h to flip loaded texture's on the y-axis.
        // The FileSystem::getPath(...) is part of the GitHub repository so we can find files on any IDE/platform; replace it with your own image path.
        unsigned char* data = stbi_load(FileSystem::getPath(temp).c_str(), &width, &height, &nrChannels, 0);
        if (data)
        {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            glGenerateMipmap(GL_TEXTURE_2D);
        }
        else
        {
            std::cout << "Failed to load texture " << 0 print j print std::endl;
        }
        stbi_image_free(data);
    }

    //cara left
    stbi_set_flip_vertically_on_load(false);
    for (int j = 0; j < 9; ++j) {
        string temp = "L";
        glGenTextures(1, &textures[1][j]);
        glBindTexture(GL_TEXTURE_2D, textures[1][j]);
        // set the texture wrapping parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        // set texture filtering parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // load image, create texture and generate mipmaps
        int width, height, nrChannels;
        temp += to_string(j);
        temp += ".jpg";
      //  cout print temp print(1 * 9) + j print  endl;

        // tell stb_image.h to flip loaded texture's on the y-axis.
        // The FileSystem::getPath(...) is part of the GitHub repository so we can find files on any IDE/platform; replace it with your own image path.
        unsigned char* data = stbi_load(FileSystem::getPath(temp).c_str(), &width, &height, &nrChannels, 0);
        if (data)
        {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            glGenerateMipmap(GL_TEXTURE_2D);
        }
        else
        {
            std::cout << "Failed to load texture " << 0 print j print std::endl;
        }
        stbi_image_free(data);
    }


    stbi_set_flip_vertically_on_load(true);
    //cara up
    for (int j = 0; j < 9; ++j) {
        string temp = "U";
        glGenTextures(1, &textures[2][j]);
        glBindTexture(GL_TEXTURE_2D, textures[2][j]);
        // set the texture wrapping parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        // set texture filtering parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // load image, create texture and generate mipmaps
        int width, height, nrChannels;
        temp += to_string(j);
        temp += ".jpg";
        //cout print temp print(2 * 9) + j print  endl;

        // tell stb_image.h to flip loaded texture's on the y-axis.
        // The FileSystem::getPath(...) is part of the GitHub repository so we can find files on any IDE/platform; replace it with your own image path.
        unsigned char* data = stbi_load(FileSystem::getPath(temp).c_str(), &width, &height, &nrChannels, 0);
        if (data)
        {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            glGenerateMipmap(GL_TEXTURE_2D);
        }
        else
        {
            std::cout << "Failed to load texture " << 0 print j print std::endl;
        }
        stbi_image_free(data);
    }

    stbi_set_flip_vertically_on_load(true);
    //cara back
    for (int j = 0; j < 9; ++j) {
        string temp = "B";
        glGenTextures(1, &textures[3][j]);
        glBindTexture(GL_TEXTURE_2D, textures[3][j]);
        // set the texture wrapping parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        // set texture filtering parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // load image, create texture and generate mipmaps
        int width, height, nrChannels;
        temp += to_string(j);
        temp += ".jpg";
        //cout print temp print(3 * 9) + j print  endl;

        // tell stb_image.h to flip loaded texture's on the y-axis.
        // The FileSystem::getPath(...) is part of the GitHub repository so we can find files on any IDE/platform; replace it with your own image path.
        unsigned char* data = stbi_load(FileSystem::getPath(temp).c_str(), &width, &height, &nrChannels, 0);
        if (data)
        {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            glGenerateMipmap(GL_TEXTURE_2D);
        }
        else
        {
            std::cout << "Failed to load texture " << 0 print j print std::endl;
        }
        stbi_image_free(data);
    }

    stbi_set_flip_vertically_on_load(true);
    //cara right
    for (int j = 0; j < 9; ++j) {
        string temp = "R";
        glGenTextures(1, &textures[4][j]);
        glBindTexture(GL_TEXTURE_2D, textures[4][j]);
        // set the texture wrapping parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        // set texture filtering parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // load image, create texture and generate mipmaps
        int width, height, nrChannels;
        temp += to_string(j);
        temp += ".jpg";
        //cout print temp print (4 * 9) + j print  endl;

        // tell stb_image.h to flip loaded texture's on the y-axis.
        // The FileSystem::getPath(...) is part of the GitHub repository so we can find files on any IDE/platform; replace it with your own image path.
        unsigned char* data = stbi_load(FileSystem::getPath("wall.jpg").c_str(), &width, &height, &nrChannels, 0);
        if (data)
        {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            glGenerateMipmap(GL_TEXTURE_2D);
        }
        else
        {
            std::cout << "Failed to load texture " << 0 print j print std::endl;
        }
        stbi_image_free(data);
    }

    stbi_set_flip_vertically_on_load(true);
    //cara down
    for (int j = 0; j < 9; ++j) {
        string temp = "D";
        glGenTextures(1, &textures[5][j]);
        glBindTexture(GL_TEXTURE_2D, textures[5][j]);
        // set the texture wrapping parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        // set texture filtering parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // load image, create texture and generate mipmaps
        int width, height, nrChannels;
        temp += to_string(j);
        temp += ".jpg";
        //cout print temp print (5*9)+j print endl;

        // tell stb_image.h to flip loaded texture's on the y-axis.
        // The FileSystem::getPath(...) is part of the GitHub repository so we can find files on any IDE/platform; replace it with your own image path.
        unsigned char* data = stbi_load(FileSystem::getPath(temp).c_str(), &width, &height, &nrChannels, 0);
        if (data)
        {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            glGenerateMipmap(GL_TEXTURE_2D);
        }
        else
        {
            std::cout << "Failed to load texture " << 0 print j print std::endl;
        }
        stbi_image_free(data);
    }

    // -------------------------------------------------------------------------------------------
    ourShader.use(); // don't forget to activate/use the shader before setting uniforms!
    // either set it manually like so:
   // glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), 0);
    // or set it via the texture class
    //ourShader.setInt("texture2", 1);
    //ourShader.setInt("texture2", 1);
   // ourShader.setInt("texture3", 2);
    int fcont = 0;
    std::cout << "M activa la animacion de explocion " << std::endl;
    std::cout <<"la camara se mueve con W, A,S,D y con el mouse" << std::endl;
    std::cout << "Movemos las caras con:" << std::endl;
    std::cout << "::L rota la cara izquierda::" << std::endl;
    std::cout << "::R rota la cara derecha::" << std::endl;
    std::cout << "::C rota la cara inferior::" << std::endl;
    std::cout << "::U rota la cara superior::" << std::endl;
    std::cout << "::B rota la cara trasera::" << std::endl;
    std::cout << "::F rota la cara frontal::" << std::endl;
    std::cout << "Q Resuelve el cubo" << std::endl;
    std::cout << "Z Rota la camara verticalmente para ver todos las caras" << std::endl;
    std::cout << "X Rota la camara horizontalmente para ver todos las caras" << std::endl;
    std::cout << "P Activa la animacion de expandir los cubos al momento de girar la camada" << std::endl;
    


    // render loop

    // -----------


    for (int t = 0; t < 6; t++) {
        for (int y = 0; y < 9; y++) {
            glActiveTexture(GL_TEXTURE0 + ((t * 9) + y));
            glBindTexture(GL_TEXTURE_2D, textures[t][y]);
            //cout print "texture" print((t * 9) + y) print "enlazada a " print "(" print t print "," print y print ")" print endl;
        }
    }

    while (!glfwWindowShouldClose(window))
    {
        // per-frame time logic
        // --------------------
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);

        // render
        // ------
        fcont++;
        if (fcont % 20 == 0)
            glClearColor((GLfloat)(rand() % 100) / 100.0f, (GLfloat)(rand() % 100) / 100.0f, (GLfloat)(rand() % 100) / 100.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // activate shader
        ourShader.use();

        // pass projection matrix to shader (note that in this case it could change every frame)
        glm::mat4 projection = glm::perspective(glm::radians(fov), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        ourShader.setMat4("projection", projection);

        // camera/view transformation
        glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
        if (valicamara % 2 != 0) {
            view = glm::rotate(view, (float)glfwGetTime(), glm::vec3(1.0f, 0.0f, 0.0f));
        }

        if (valicamara2 % 2 != 0) {
            view = glm::rotate(view, (float)glfwGetTime(), glm::vec3(0.0f, 1.0f, 0.0f));
        }
        ourShader.setMat4("view", view);


       


        ourShader.setFloat("mixValue", mixValue);
        ourShader.use();


        //////////////////////////////ROTACION///////////////////////////////
        /////////////////////////////////////////////////////////////////////

        if (valiInicial mayora 0 && valiInicial menora limite+(limite/2)) {



            for (int i = 0; i < NCUBES; ++i) {
                map<int, int>::iterator q = indice_tag.find(i);

                glBindVertexArray(VAO[q->second]);
                // cout print q->second;
                glm::mat4 model = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
                if(valiInicial menora (limite/2) ){
                    if (i % 6 == 1)model = glm::translate(model, glm::vec3(0.0f, -0.1f * valiInicial, 0.0f));
                    if (i % 6 == 0)model = glm::translate(model, glm::vec3(0.0f, 0.1f * valiInicial, 0.0f));
                    if (i % 6 == 3)model = glm::translate(model, glm::vec3(0.1f * valiInicial, 0.0f, 0.0f));
                    if (i % 6 == 2)model = glm::translate(model, glm::vec3(-0.1f * valiInicial, 0.0f, 0.0f));
                    if (i % 6 == 4)model = glm::translate(model, glm::vec3(0.0f, 0.0f, -0.1f * valiInicial));
                    if (i % 6 == 5)model = glm::translate(model, glm::vec3(0.0f, 0.0f, 0.1f * valiInicial));

                }
                else {
                    if (i % 6 == 1)model = glm::translate(model, glm::vec3(0.0f, -0.04f * (limite + (limite / 2) - valiInicial), 0.0f));
                    if (i % 6 == 0)model = glm::translate(model, glm::vec3(0.0f, 0.04f * (limite + (limite / 2) - valiInicial), 0.0f));
                    if (i % 6 == 3)model = glm::translate(model, glm::vec3(0.04f * (limite + (limite / 2) - valiInicial), 0.0f, 0.0f));
                    if (i % 6 == 2)model = glm::translate(model, glm::vec3(-0.04f * (limite + (limite / 2) - valiInicial), 0.0f, 0.0f));
                    if (i % 6 == 4)model = glm::translate(model, glm::vec3(0.0f, 0.0f, -0.04f * (limite + (limite / 2) - valiInicial)));
                    if (i % 6 == 5)model = glm::translate(model, glm::vec3(0.0f, 0.0f, 0.04f * (limite + (limite / 2) - valiInicial)));

                    model = glm::rotate(model, -0.1f * (limite + (limite / 2) - valiInicial), glm::vec3(1.0f, 1.0f, 1.0f));
                    if (i % 7 == 1) model = glm::rotate(model, toRadians(2 * (limite + (limite / 2) - valiInicial)), glm::vec3(0.0f, 0.0f, 1.0f));
                    if (i % 7 == 0) model = glm::rotate(model, toRadians(2 * (limite + (limite / 2) - valiInicial)), glm::vec3(0.0f, 1.0f, 0.0f));
                    if (i % 7 == 2) model = glm::rotate(model, toRadians(2 * (limite + (limite / 2)- valiInicial)), glm::vec3(1.0f, 0.0f, 0.0f));
                    if (i % 7 == 3) model = glm::rotate(model, toRadians(2 * (limite + (limite / 2) - valiInicial)), glm::vec3(1.0f, 1.0f, 0.0f));
                    if (i % 7 == 4) model = glm::rotate(model, toRadians(2 * (limite + (limite / 2) - valiInicial)), glm::vec3(0.0f, 1.0f, 1.0f));
                    if (i % 7 == 5) model = glm::rotate(model, toRadians(2 * (limite + (limite / 2) - valiInicial)), glm::vec3(1.0f, 0.0f, 1.0f));
                    if (i % 7 == 6) model = glm::rotate(model, toRadians(2 * (limite + (limite / 2) - valiInicial)), glm::vec3(1.0f, 1.0f, 1.0f));
                }

                ourShader.setMat4("model", model);



                if (q->second < 9) {
                    for (int m = 0; m < 6; ++m) {

                        ourShader.use();
                        if (m == 1) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), q->second);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }
                        else if (m == 2) {
                            //cout print "entrando m=2 ..." print (q->second) + 7  print endl;
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 7);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }



                        else if (m == 4) {
                            // cout print "entrando m=4 ..." print(q->second) + 39  print endl;
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 39);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }

                        /*else if (m == 3) {
                            cout print "entrando m=3 ..." print(q->second) + 36  print endl;
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 36);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }*/

                        else if (m == 5) {
                            // cout print "entrando m=5 ..." print(q->second) + 18  print endl;
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 18);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }

                        //draw 2 triangles making up this face
                        glDrawArrays(GL_TRIANGLES, (m * 6), 6);

                    }
                }
                else if (q->second menora 18) {
                    for (int m = 0; m < 6; ++m) {

                        ourShader.use();

                        if (m == 2) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) - 1);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }

                        /* else if (m == 3) {
                             glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 28);
                             glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                         }*/

                        else if (m == 4) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 33);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }

                        else if (m == 5) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 12);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }

                        glDrawArrays(GL_TRIANGLES, (m * 6), 6);

                    }

                }

                else if (q->second menora 27) {
                    for (int m = 0; m < 6; ++m) {

                        ourShader.use();

                        if (m == 0) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 9);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }
                        else if (m == 2) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) - 9);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }

                        /* else if (m == 3) {
                             glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) +20);
                             glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                         }*/

                        else if (m == 4) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 27);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }
                        else if (m == 5) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 6);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }

                        glDrawArrays(GL_TRIANGLES, (m * 6), 6);

                    }




                }

            }
            if (valiInicial == limite+(limite/2)) {
                valiInicial = 0;
            }
            valiInicial++;


        }



        else {



            for (int j = 0; j < NCUBES; ++j) {
                glBindVertexArray(VAO[j]);
                glBindBuffer(GL_ARRAY_BUFFER, VBO[j]);
                glBufferData(GL_ARRAY_BUFFER, sizeof(punteros[j][0]) * 288, punteros[j], GL_STATIC_DRAW);

                // position attribute
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
                glEnableVertexAttribArray(0);
                // color attribute
                glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
                glEnableVertexAttribArray(1);
                // texture coord attribute
                glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
                glEnableVertexAttribArray(2);
            }

            if (fcont % 20 == 0) {
                if (!solucion.empty())
                {
                    if (solucion.front() == "F'") { valiFront = 1; }
                    if (solucion.front() == "B'") { valiBack = 1; }
                    if (solucion.front() == "L'") { valiLeft = 1; }
                    if (solucion.front() == "R'") { valiRight = 1; }
                    if (solucion.front() == "U'") { valiUp = 1; }
                    if (solucion.front() == "D'") { valiDown = 1; }

                    solucion.pop();

                }
            }



            for (int i = 0; i < NCUBES; ++i) {
                map<int, int>::iterator q = indice_tag.find(i);

                glBindVertexArray(VAO[q->second]);
                // cout print q->second;
                glm::mat4 model = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first

                if (valiLeft mayora 0) {
                    model = rotarLeft(valiLeft, i, model);
                }

                if (valiFront mayora 0) {
                    model = rotarFront(valiFront, i, model);
                }

                if (valiUp mayora 0) {
                    model = rotarUp(valiUp, i, model);
                }

                if (valiRight mayora 0) {
                    model = rotarRight(valiRight, i, model);
                }

                if (valiDown mayora 0) {
                    model = rotarDown(valiDown, i, model);
                }

                if (valiBack mayora 0) {
                    model = rotarBack(valiBack, i, model);
                }

                ourShader.setMat4("model", model);



                if (q->second < 9) {
                    for (int m = 0; m < 6; ++m) {

                        ourShader.use();
                        if (m == 1) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), q->second);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }
                        else if (m == 2) {
                            //cout print "entrando m=2 ..." print (q->second) + 7  print endl;
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 7);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }



                        else if (m == 4) {
                            // cout print "entrando m=4 ..." print(q->second) + 39  print endl;
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 39);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }

                        /*else if (m == 3) {
                            cout print "entrando m=3 ..." print(q->second) + 36  print endl;
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 36);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }*/

                        else if (m == 5) {
                            // cout print "entrando m=5 ..." print(q->second) + 18  print endl;
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 18);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }

                        //draw 2 triangles making up this face
                        glDrawArrays(GL_TRIANGLES, (m * 6), 6);

                    }
                }
                else if (q->second menora 18) {
                    for (int m = 0; m < 6; ++m) {

                        ourShader.use();

                        if (m == 2) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) - 1);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }

                        /* else if (m == 3) {
                             glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 28);
                             glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                         }*/

                        else if (m == 4) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 33);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }

                        else if (m == 5) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 12);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }

                        glDrawArrays(GL_TRIANGLES, (m * 6), 6);

                    }

                }

                else if (q->second menora 27) {
                    for (int m = 0; m < 6; ++m) {

                        ourShader.use();

                        if (m == 0) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 9);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }
                        else if (m == 2) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) - 9);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }

                        /* else if (m == 3) {
                             glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) +20);
                             glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                         }*/

                        else if (m == 4) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 27);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }
                        else if (m == 5) {
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), (q->second) + 6);
                            glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 53);
                        }

                        glDrawArrays(GL_TRIANGLES, (m * 6), 6);

                    }




                }
                if (q->second % 3 == 0) {
                    ourShader.use();
                    //cout print q->second print endl;
                    glUniform1i(glGetUniformLocation(ourShader.ID, "texture1"), 4);
                    glUniform1i(glGetUniformLocation(ourShader.ID, "texture2"), 4);
                    glDrawArrays(GL_TRIANGLES, 18, 6);

                }

            }
            // glDrawArrays(GL_TRIANGLES, 0, 36);


            if (valisolver % 2 != 0) {
                rtrim(movimientos);
                //cout print "Movimientos que realizamos:" print endl;
                //cout print movimientos print endl;
                OptimalSolver os(6);
                os.solve(movimientos);
                movimientos = "";
                solucion = moves_(os.solution);
                valisolver += 1;
                /*while (!solucion.empty()) {
                    cout print "estee " print solucion.front() print endl;
                    solucion.pop();
                }*/


            }

        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }


    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------
    glDeleteVertexArrays(4, VAO);
    glDeleteBuffers(4, VBO);


    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
   
    glfwTerminate();
    return 0;
   
    
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    float cameraSpeed = 2.5 * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraPos += cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos -= cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) {
        valiLeft = 1;

    }
    if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) {
        valiFront = 1;

    }
    if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS){
        valiUp = 1;

    }
    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS){
        valiRight = 1;

    }
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS){
        valiDown = 1;

    }
    if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS) {
        valiBack = 1;

    }

  
}


Vertice mitad(Vertice a, Vertice b) {
    Vertice resul;
    resul.x = (a.x + b.x) / 2;
    resul.y = (a.y + b.y) / 2;
    resul.z = (a.z + b.z) / 2;
    return resul;
}

void Generar3triangulos(Triangulo triangulo) {
    Vertice mtd_a_b = mitad(triangulo.a, triangulo.b);
    Vertice mtd_b_c = mitad(triangulo.b, triangulo.c);
    Vertice mtd_a_c = mitad(triangulo.a, triangulo.c);
    mtd_a_b.a = mtd_b_c.a = mtd_a_c.a = color_actual[0];
    mtd_a_b.b = mtd_b_c.b = mtd_a_c.b = color_actual[1];
    mtd_a_b.c = mtd_b_c.c = mtd_a_c.c = color_actual[2];
    color_actual[0] = (GLfloat)(rand() % 100) / 100.0f;
    color_actual[1] = (GLfloat)(rand() % 100) / 100.0f;
    color_actual[2] = (GLfloat)(rand() % 100) / 100.0f;
    Triangulo nuevo1(triangulo.a, mtd_a_b, mtd_a_c);
    Triangulo nuevo2(triangulo.b, mtd_b_c, mtd_a_b);
    Triangulo nuevo3(triangulo.c, mtd_b_c, mtd_a_c);
    act_Vert_1_tr(nuevo1);
    act_Vert_1_tr(nuevo2);
    act_Vert_1_tr(nuevo3);
}


void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
 

    if (key == GLFW_KEY_Q && action == GLFW_PRESS) {
        valisolver += 1;
    }

    if (key == GLFW_KEY_L && action == GLFW_PRESS) {
        movimientos += "L' ";
    }
    if (key == GLFW_KEY_F && action == GLFW_PRESS) {

        movimientos += "F' ";
    }
    if (key == GLFW_KEY_U && action == GLFW_PRESS) {

        movimientos += "U' ";
    }
    if (key == GLFW_KEY_R && action == GLFW_PRESS) {

        movimientos += "R' ";
    }
    if (key == GLFW_KEY_C && action == GLFW_PRESS) {

        movimientos += "D' ";
    }
    if (key == GLFW_KEY_B && action == GLFW_PRESS) {

        movimientos += "B' ";
    }

    if (key == GLFW_KEY_Z && action == GLFW_PRESS) {

       valicamara += 1;
    }

    if (key == GLFW_KEY_X && action == GLFW_PRESS) {

        valicamara2 += 1;
    }


    if (key == GLFW_KEY_P && action == GLFW_PRESS) {
        valiAnimacion ^= 1;
    }



    if (key == GLFW_KEY_M && action == GLFW_PRESS) {
        valiInicial =1;
    }



    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
    {
        mixValue += 0.05f; // change this value accordingly (might be too slow or too fast based on system hardware)
        if (mixValue >= 1.0f)
            mixValue = 1.0f;

        cout << mixValue << endl;
    }
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
    {
        mixValue -= 0.05f; // change this value accordingly (might be too slow or too fast based on system hardware)
        if (mixValue <= 0.0f)
            mixValue = 0.0f;
        cout << mixValue << endl;
    }
}






// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
    lastX = xpos;
    lastY = ypos;

    float sensitivity = 0.1f; // change this value to your liking
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    yaw += xoffset;
    pitch += yoffset;

    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (pitch > 89.0f)
        pitch = 89.0f;
    if (pitch < -89.0f)
        pitch = -89.0f;

    glm::vec3 front;
    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.y = sin(glm::radians(pitch));
    front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    cameraFront = glm::normalize(front);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    fov -= (float)yoffset;
    if (fov < 1.0f)
        fov = 1.0f;
    if (fov > 45.0f)
        fov = 45.0f;
}

