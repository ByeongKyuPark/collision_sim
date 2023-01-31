#define GLFW_INCLUDE_NONE
#include <graphics/opengl/glad/glad.h>

#include <graphics/renderer.h>
#include <iostream>
#include <typeinfo>
#include "graphics/imageID.h"

#define STB_IMAGE_IMPLEMENTATION
#include "../include/graphics/stb_image/stb_image.h"

using namespace Graphics;

GLuint Graphics::textures[NUM_IMAGES];

void Graphics::Renderer::SetupTextures(){
    for (int i = 0; i < NUM_IMAGES; ++i) {
        glGenTextures(NUM_IMAGES, textures);
        int imgWidth, imgHeight, numComponents;
        unsigned char* imgData = stbi_load(imgFileName[i], &imgWidth, &imgHeight, &numComponents, 3);
        if (!imgData)
        {
            std::cerr << stbi_failure_reason() << std::endl;
            exit(1);
        }

        glActiveTexture(GL_TEXTURE0+i);
        glBindTexture(GL_TEXTURE_2D, textures[i]);

        glEnable(GL_TEXTURE_2D);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 2);
        if (numComponents == 3)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, imgWidth, imgHeight, 0,
                GL_RGB, GL_UNSIGNED_BYTE, imgData);
        else
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, imgWidth, imgHeight, 0,
                GL_RGBA, GL_UNSIGNED_BYTE, imgData);

        free(imgData);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 4);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

Renderer::Renderer()
    : windowWidth(WIN_WIDTH), windowHeight(WIN_HEIGHT), window{nullptr}
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);//3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);//3
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    window = glfwCreateWindow(windowWidth, windowHeight, "RigidBody Physics Sandbox", NULL, NULL);
    if (!window){
        std::cerr << "glfwCreateWindow error \n";
        glfwTerminate();
        exit(1);
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)){
        std::cerr << "gladLoadGLLoader error\n";
        glfwTerminate();
        exit(1);
    }

    objectShader = Shader(
        "./shaders/object_vertex.glsl",
        "./shaders/object_fragment.glsl"
    );
    objectShaderNoTexture = Shader(
        "./shaders/object_vertex_no_texture.glsl",
        "./shaders/object_fragment_no_texture.glsl"
    );

    glGenVertexArrays(1, &backgroundVAO);
    glBindVertexArray(backgroundVAO);

    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        sizeof(float) * GRID_VERTICES.size(),
        &GRID_VERTICES[0],
        GL_STATIC_DRAW
    );

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);

    SetupTextures();

    glViewport(0, 0, windowWidth, windowHeight);
    glClearColor(0.2f, 0.9f, 0.25f, 1.0f);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LINE_SMOOTH);
}

Renderer::~Renderer(){
    for (auto& shape : shapes)
    {
        delete shape.second;
    }

    glfwTerminate();
}

GLFWwindow* Renderer::GetWindow() const
{
    return window;
}

unsigned int Renderer::GetTextureBufferID() const{
    return textureBufferID;
}

Shape* Renderer::addShape(unsigned int id, Geometry geometry, const Vector& scl)
{
    Shape* newShape{nullptr};

    if (geometry == SPHERE)
        newShape = new Sphere(scl);
    else if (geometry == BOX)
        newShape = new Box(scl);

    shapes[id] = newShape;
    return newShape;
}

void Renderer::RenderObject(
    Geometry type,
    unsigned int id,
    unsigned int  imageId,
    float modelMatrix[],
    const Camera& camera
)
{
    /* 변환 행렬 설정 */
    glm::mat4 model = glm::make_mat4(modelMatrix);
    glm::mat4 view = camera.GetViewMatrix();
    glm::mat4 projection = glm::perspective(
        glm::radians(camera.GetFov()),
        ((float) windowWidth) / windowHeight,
        PERSPECTIVE_NEAR,
        PERSPECTIVE_FAR
    );

    objectShader.use();
    objectShader.setMat4("model", model);
    objectShader.setMat4("view", view);
    objectShader.setMat4("projection", projection);
    objectShader.setVec3("viewPos", camera.GetPosition());

    Shape *objectShape = shapes.find(id)->second;
    objectShader.setInt("texture1", imageId);

    if (drawFramesOnly==false) {
        glBindVertexArray(objectShape->polygonVAO);
        if (type == Geometry::BOX) {
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
        else {
            glDrawElements(GL_TRIANGLES, (objectShape->indexBuffer)->size(), GL_UNSIGNED_INT, (void*)0);
        }
    }

    if (id != 0)
    {
        glm::vec3 frameColor(0.7f, 0.7f, 0.7f);
        objectShader.setVec3("objectColor", frameColor);
        glBindVertexArray(objectShape->frameVAO);
        glDrawElements(GL_LINE_STRIP, (objectShape->indexBuffer)->size(), GL_UNSIGNED_INT, (void*)nullptr);
        if (typeid(*objectShape) == typeid(Sphere))
        {
            for (int i = 0; i < 3; ++i)
            {
                model = glm::rotate(model, glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
                objectShader.setMat4("model", model);
                glDrawElements(GL_LINE_STRIP, (objectShape->indexBuffer)->size(), GL_UNSIGNED_INT, (void*)nullptr);
            }
            model = glm::make_mat4(modelMatrix);
            model = glm::rotate(model, glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            objectShader.setMat4("model", model);
            glDrawElements(GL_LINE_STRIP, (objectShape->indexBuffer)->size(), GL_UNSIGNED_INT, (void*)nullptr);
            model = glm::rotate(model, glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            objectShader.setMat4("model", model);
            glDrawElements(GL_LINE_STRIP, (objectShape->indexBuffer)->size(), GL_UNSIGNED_INT, (void*)nullptr);
        }
    }

    glBindVertexArray(0);
}

void Renderer::RenderBackground(const Camera&camera){
    glm::mat4 view = camera.GetViewMatrix();
    glm::mat4 projection = glm::perspective(
        glm::radians(camera.GetFov()),
        ((float) windowWidth) / windowHeight,
        PERSPECTIVE_NEAR,
        PERSPECTIVE_FAR
    );

    objectShaderNoTexture.use();
    objectShaderNoTexture.setMat4("view", view);
    objectShaderNoTexture.setMat4("projection", projection);
    objectShaderNoTexture.setVec3("objectColor", glm::vec3(0.3f, 0.95f, 0.35));
    objectShaderNoTexture.setVec3("viewPos", camera.GetPosition());

    glBindVertexArray(0);
}

void Renderer::UpdateWindowSize()
{
    glfwGetFramebufferSize(window, &windowWidth, &windowHeight);
    glViewport(0, 0, windowWidth, windowHeight);
}

void Renderer::BindSceneFrameBuffer()
{
    glBindFramebuffer(GL_FRAMEBUFFER, sceneFrameBufferID);
}

void Renderer::BindDefaultFrameBuffer()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
void Renderer::SetWindowViewport()
{
    glViewport(0, 0, windowWidth, windowHeight);
}
