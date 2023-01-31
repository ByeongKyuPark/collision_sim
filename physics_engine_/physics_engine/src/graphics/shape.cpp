//#define GLFW_INCLUDE_NONE
#include <graphics/opengl/glad/glad.h>
#include <gl/GL.h>

//#define GLFW_INCLUDE_NONE
//#include <opengl/glad/glad.h>
#include <opengl/glm/glm.hpp>

#include <graphics/shape.h>
#include <cmath>
#include <cstdarg>
#include <iostream>

using namespace Graphics;

using std::shared_ptr;
using std::make_shared;

const float PI = 3.141592f;

shared_ptr<IndexBufferType> Box::boxIndexBuffer = std::make_shared<IndexBufferType>(
    IndexBufferType{
        0, 1, 2, 3, 0,      
        4, 5, 6, 7, 4,
        5, 6, 2, 1, 5,
        4, 0, 3, 7, 4, 
        7, 3, 2, 6, 7,
        4, 5, 1, 0, 4
    }
);
shared_ptr<FrameIndexBufferType> Box::boxFrameIndexBuffer = std::make_shared<FrameIndexBufferType>(
    IndexBufferType{
        2, 1, 7, 8
        , 14, 5, 2,
        1, 0, 5,
        14, 13, 0,
        1, 7, 13,
        7, 8, 4, 1
    }
);

shared_ptr<IndexBufferType> Sphere::sphereIndexBuffer=nullptr;
shared_ptr<FrameIndexBufferType> Sphere::sphereFrameIndexBuffer=nullptr;

void Shape::CreateVertexArray()
{
    glGenVertexArrays(1, &polygonVAO);
    glBindVertexArray(polygonVAO);
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        vertexBuffer.size() * sizeof(Vertex),
        &vertexBuffer[0],
        GL_STATIC_DRAW
    );

    GLuint ebo;
    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        (* indexBuffer).size() * sizeof(unsigned),
        &(* indexBuffer)[0],
        GL_STATIC_DRAW
    );

    for (int i = 0; i < NumAttribs; ++i){
        glEnableVertexAttribArray(vertexLayout[i].location);
        glVertexAttribPointer(vertexLayout[i].location, vertexLayout[i].size,GL_FLOAT,GL_FALSE,sizeof(Vertex), (void*)vertexLayout[i].offset);
    }

    glBindVertexArray(0);

    glGenVertexArrays(1, &frameVAO);
    glBindVertexArray(frameVAO);
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        vertexBuffer.size() * sizeof(Vertex),
        &vertexBuffer[0],
        GL_STATIC_DRAW
    );

    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        (* frameIndexBuffer).size() * sizeof(unsigned),
        &(*frameIndexBuffer)[0],
        GL_STATIC_DRAW
    );

    for (int i = 0; i < NumAttribs; ++i){
        glEnableVertexAttribArray(vertexLayout[i].location);
        glVertexAttribPointer(vertexLayout[i].location, vertexLayout[i].size, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)vertexLayout[i].offset);
    }
}

Box::Box(const Vector& scale)
    : Shape(boxIndexBuffer,boxFrameIndexBuffer){
    CreateVertices(scale);
    CreateVertexArray();
}

void Box::CreateVertices(const Vector& scl){
    vertexBuffer.clear();

    //https://gamedev.stackexchange.com/questions/119823/trouble-applying-a-texture-to-a-cube
    const GLfloat vertices[] = {
      -scl.x, -scl.y, scl.z, 
      scl.x, -scl.y, scl.z, 
      scl.x, scl.y, scl.z,
      -scl.x, -scl.y, scl.z, 
      scl.x, scl.y, scl.z,
      -scl.x, scl.y, scl.z,
      //---------------------
      scl.x, -scl.y, scl.z, 
      scl.x, -scl.y, -scl.z, 
      scl.x, scl.y, -scl.z,
      scl.x, -scl.y, scl.z,
      scl.x, scl.y, -scl.z,
      scl.x, scl.y, scl.z,
      //---------------------
      scl.x, -scl.y, -scl.z,
      -scl.x, -scl.y, -scl.z, 
      -scl.x, scl.y, -scl.z,
      scl.x, -scl.y, -scl.z, 
      -scl.x, scl.y, -scl.z, 
      scl.x, scl.y, -scl.z,
     //---------------------
      -scl.x, -scl.y, -scl.z, 
      -scl.x, -scl.y, scl.z, 
      -scl.x, scl.y, scl.z,
      -scl.x, -scl.y, -scl.z, 
      -scl.x, scl.y, scl.z, 
      -scl.x, scl.y, -scl.z,
      //---------------------
      -scl.x, scl.y, scl.z, 
      scl.x, scl.y, scl.z, 
      scl.x, scl.y, -scl.z,
      -scl.x, scl.y, scl.z, 
      scl.x, scl.y, -scl.z, 
      -scl.x, scl.y, -scl.z,
      //---------------------       //bottom
      -scl.x, -scl.y, scl.z, 
      -scl.x, -scl.y, -scl.z, 
      scl.x, -scl.y, -scl.z,
      -scl.x, -scl.y, scl.z, 
      scl.x, -scl.y, -scl.z, 
      scl.x, -scl.y, scl.z
    };

    static constexpr GLfloat uvCoord[] = {
      0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,
      0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f,

      0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,
      0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f,

      0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,
      0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f,

      0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,
      0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f,

      0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,
      0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f,

      0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,
      0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f
    };

    constexpr int NumBoxVertices = 36;
    for (int i{}; i < NumBoxVertices; ++i) {
        Vertex v;
        v.pos = {vertices[3*i],vertices[3*i+1],vertices[3*i+2], };
        v.nrm = { 0.f,0.f,1.f };//temp
        v.uv = { uvCoord[i*2],uvCoord[i*2+1]};
        vertexBuffer.emplace_back(v);
    }
}

Sphere::Sphere(const Vector& scale)
{
    CreateVertices(scale);
    if (sphereIndexBuffer == nullptr || sphereFrameIndexBuffer == nullptr) {
        CreateIndices();
    }
    indexBuffer = sphereIndexBuffer;
    frameIndexBuffer = sphereFrameIndexBuffer;
    CreateVertexArray();
}

//http://www.songho.ca/opengl/gl_sphere.html
void Sphere::CreateVertices(const Vector& scale)
{
    float x, y, z, xy;
    float nx, ny, nz, lengthInv = 1.0f / scale.x;    // vertex normal
    float s, t;                                     // vertex texCoord

    float sectorStep = 2 * PI / SECTOR_CNT;
    float stackStep = PI / STACK_CNT;
    float sectorAngle, stackAngle;

    vertexBuffer.clear();
    for(int i = 0; i <= STACK_CNT; ++i)
    {
        stackAngle = PI / 2 - i * stackStep;        // starting from pi/2 to -pi/2
        xy = scale.x * cosf(stackAngle);             // r * cos(u)
        z = scale.x * sinf(stackAngle);              // r * sin(u)

        // add (sectorCount+1) vertices per stack
        // the first and last vertices have same position and normal, but different tex coords
        for(int j = 0; j <= SECTOR_CNT; ++j)
        {
            sectorAngle = j * sectorStep;           // starting from 0 to 2pi

            Vertex v;

            // vertex position (a, b, c)
            x = xy * cosf(sectorAngle);             // r * cos(u) * cos(v)
            y = xy * sinf(sectorAngle);             // r * cos(u) * sin(v)

            v.pos = {x,y,z};
            v.nrm = glm::normalize(v.pos);
            v.uv = { (float)j / SECTOR_CNT, (float)i / STACK_CNT};

            vertexBuffer.push_back(v);
        }
    }
}

void Sphere::CreateIndices()
{
    IndexBufferType sphereIdxBuffer;
    FrameIndexBufferType sphereFrameIdxBuffer;

    int k1, k2;
    for(int i = 0; i < STACK_CNT; ++i)
    {
        k1 = i * (SECTOR_CNT + 1);     // beginning of current stack
        k2 = k1 + SECTOR_CNT + 1;      // beginning of next stack

        for(int j = 0; j < SECTOR_CNT; ++j, ++k1, ++k2)
        {
            // 2 triangles per sector excluding first and last stacks
            // k1 => k2 => k1+1
            if(i != 0)
            {
                sphereIdxBuffer.push_back(k1);
                sphereIdxBuffer.push_back(k2);
                sphereIdxBuffer.push_back(k1 + 1);
            }

            // k1+1 => k2 => k2+1
            if(i != (STACK_CNT-1))
            {
                sphereIdxBuffer.push_back(k1 + 1);
                sphereIdxBuffer.push_back(k2);
                sphereIdxBuffer.push_back(k2 + 1);
            }

            // store indices for lines
            // vertical lines for all stacks, k1 => k2
        }
        sphereFrameIdxBuffer.push_back(k1);
        sphereFrameIdxBuffer.push_back(k2);
    }
    sphereIndexBuffer = make_shared<IndexBufferType>(sphereIdxBuffer);
    sphereFrameIndexBuffer = make_shared<FrameIndexBufferType>(sphereFrameIdxBuffer);
}