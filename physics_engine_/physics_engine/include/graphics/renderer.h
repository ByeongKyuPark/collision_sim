#pragma once

#include "shape.h"
#include "shader.h"
#include "camera.h"
#include "../collisionSimulator/geometry.h"
#include "../collisionSimulator/contact_info.h"
#include <GLFW/glfw3.h>
#include <unordered_map>
#include "graphics/imageID.h"

namespace Graphics
{
    extern GLuint textures[NUM_IMAGES];

    const std::vector<float> GRID_VERTICES = {
        0.0f, 0.0f, 100.0f,
        0.0f, 0.0f, -100.0f
    };
    constexpr float GRID_GAP = 1.0f;

    constexpr int WIN_WIDTH = 1280;
    constexpr int WIN_HEIGHT = 720;

    constexpr float PERSPECTIVE_NEAR = 0.1f;
    constexpr float PERSPECTIVE_FAR = 650.0f;
    
    class Renderer
    {
    public:
        typedef std::unordered_map<unsigned int, Shape*> Shapes;

        //temporarily public
        bool drawFramesOnly = false;
    private:

        Shapes shapes;

        int windowWidth, windowHeight;
        GLFWwindow *window;

        Shader objectShader;
        Shader objectShaderNoTexture;
        unsigned int backgroundVAO;
        unsigned int worldYaxisVAO;
        unsigned int sceneFrameBufferID;
        unsigned int textureBufferID;
        
        void SetupTextures();
    public:
        Renderer();
        ~Renderer();

        GLFWwindow* GetWindow() const;
        unsigned int GetTextureBufferID() const;

        Shape* addShape(unsigned int id, Geometry, const Vector& scale);

        void RenderObject(
            Geometry type,
            unsigned int id,
            unsigned int imageId,
            float modelMatrix[],
            const Camera& camera
        );
        void RenderBackground(const Camera& camera);

        void UpdateWindowSize();

        void BindSceneFrameBuffer();
        void BindDefaultFrameBuffer();

        void SetWindowViewport();

        Shader GetShader() { return objectShader; }
    };
}