#include <graphics/camera.h>
#include <graphics/opengl/glm/gtc/quaternion.hpp>
#include <iostream>

using namespace Graphics;

Camera::Camera()
{
    position = glm::vec3(14.0f, 14.0f, 14.0f);
    lookAtPoint = glm::vec3(0.0f, 0.0f, 0.0f);
    worldUp = glm::vec3(0.0f, 1.0f, 0.0f);
    fov = 45.0f;
    Update();
}

glm::mat4 Camera::GetViewMatrix() const
{
    return glm::lookAt(position, lookAtPoint, up);
}

glm::vec3 Camera::GetViewPlaneNormal() const
{
    glm::vec3 viewPlaneNormal = glm::cross(up, right);
    return glm::normalize(viewPlaneNormal);
}

void Graphics::Camera::MoveForward(){
    position += glm::cross(up, right) * MoveSensitivity;
}

void Graphics::Camera::MoveBack(){
    position -= glm::cross(up, right) * MoveSensitivity;
}

void Graphics::Camera::MoveRight(){
    position -= right * MoveSensitivity;
    lookAtPoint -= right * MoveSensitivity;
}

void Graphics::Camera::MoveLeft(){
    position += right * MoveSensitivity;
    lookAtPoint += right * MoveSensitivity;
}
void Graphics::Camera::MoveUp() {
	position += up * MoveSensitivity;
	lookAtPoint += up * MoveSensitivity;
}
void Graphics::Camera::MoveDown() {
	position -= up * MoveSensitivity;
	lookAtPoint -= up * MoveSensitivity;
}



glm::vec3 Camera::GetPosition() const
{
    return position;
}

float Camera::GetFov() const
{
    return fov;
}

void Camera::Update()
{
    right = glm::normalize(glm::cross(lookAtPoint - position, worldUp));
    up = glm::normalize(glm::cross(right, lookAtPoint - position));
}
