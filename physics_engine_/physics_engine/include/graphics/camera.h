#pragma once

#include "GLFW/glfw3.h"
#include "opengl/glm/glm.hpp"

namespace Graphics{
    class Camera
    {
        static constexpr float MoveSensitivity = 0.035f;
    protected:
        glm::vec3 position;
        glm::vec3 lookAtPoint;
        glm::vec3 up;
        glm::vec3 right;
        glm::vec3 worldUp;

        float fov;

    public:
        Camera();

        glm::mat4 GetViewMatrix() const;
        glm::vec3 GetViewPlaneNormal() const;

        void MoveForward();
        void MoveBack();
        void MoveRight();
        void MoveLeft();
        void MoveUp();
        void MoveDown();

        glm::vec3 GetPosition() const;
        float GetFov() const;

    private:
        void Update();
    };    
}