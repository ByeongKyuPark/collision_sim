#include <physics/body.h>
#include <cmath>
#include <iostream>
#include <cstdarg>
#include "collisionSimulator/collisionSimulatorInfo.h"

using namespace physics;

//https://www.slideshare.net/xtozero/game-physics-engine-development//p21
void RigidBody::Update(float dt)
{
    if (inverseMass == 0.0f) {
        return;
    }

    Vector acceleration = force * inverseMass;
    velocity += acceleration * dt;
    velocity *= powf(linearDamping, dt);

    Vector angAccel = inverseInertiaTensorWorld * torque;
    angularVelocity += angAccel * dt;
    angularVelocity *= powf(angularDamping, dt);

    if (fabsf(velocity.x) < Flt_Epsilon) {
        velocity.x = 0.0f;
    }
    if (fabsf(velocity.y) < Flt_Epsilon) {
        velocity.y = 0.0f;
    }
    if (fabsf(velocity.z) < Flt_Epsilon) {
        velocity.z = 0.0f;
    }

    position += velocity * dt;

    Quaternion dq(orientation);
    dq *= Quaternion{ 0.f, Vector{ angularVelocity * dt} };
    dq*=0.5f;
    orientation += dq;

    orientation.Normalize();
    UpdateModelToWorldMatrix();
    TransformInertiaTensor();

    force.clear();
    torque.clear();
}


void physics::RigidBody::ApplyForce() { 
	force = { 0.f,g_Gravity / inverseMass,0.f };
}

Vector RigidBody::GetAxis(int index) const
{
    if (index < 0 || index > 3)
    {
        std::cerr << "axis idx out of bound\n";
        return Vector();
    }

    Vector result(
        modelToWorldMat.entries[index],
        modelToWorldMat.entries[index + 4],
        modelToWorldMat.entries[index + 8]
    );
    result.normalize();

    return result;
}

void RigidBody::Rotate(const Quaternion& quat)
{
    Quaternion newOrientation = GetOrientation() * quat;
    newOrientation.Normalize();
    SetOrientation(newOrientation);
}

void RigidBody::UpdateModelToWorldMatrix()
{
    //scale <--- vertices are scaled already (in GenerateVertices())

    //rotation
    QuaternionToRotateMatrix();
    
    //translation
    modelToWorldMat.entries[3] = position.x;
    modelToWorldMat.entries[7] = position.y;
    modelToWorldMat.entries[11] = position.z;
}

void physics::RigidBody::QuaternionToRotateMatrix(){
    //transposed
    const float aa = orientation.a * orientation.a;
    const float ab = orientation.a * orientation.b;
    const float ac = orientation.a * orientation.c;
    const float ad = orientation.a * orientation.d;
    const float bb = orientation.b * orientation.b;
    const float bc = orientation.b * orientation.c;
    const float bd = orientation.b * orientation.d;
    const float cc = orientation.c * orientation.c;
    const float cd = orientation.c * orientation.d;

    modelToWorldMat.entries[0] = 1.f - 2.f * (bb + cc);
    modelToWorldMat.entries[1] = 2.f * (ab - cd);
    modelToWorldMat.entries[2] = 2.f * (ac + bd);
    modelToWorldMat.entries[4] = 2.f * (ab + cd);
    modelToWorldMat.entries[5] = 1.f - 2.f * (aa + cc);
    modelToWorldMat.entries[6] = 2.f * (bc - ad);
    modelToWorldMat.entries[8] = 2.f * (ac - bd);
    modelToWorldMat.entries[9] = 2.f * (bc + ad);
    modelToWorldMat.entries[10] = 1.f - 2.f * (aa + bb);
}

void RigidBody::TransformInertiaTensor(){
    Matrix3 rotationMatrix;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            rotationMatrix.entries[3*i + j] = modelToWorldMat.entries[4*i + j];

    inverseInertiaTensorWorld = (rotationMatrix * inverseInertiaTensor) * rotationMatrix.Transpose();
}

void RigidBody::SetMass(float value)
{
    inverseMass = 1.0f / value;
}

void RigidBody::SetInverseMass(float value)
{
    inverseMass = value;
}

void RigidBody::SetInertiaTensor(const Matrix3& mat)
{
    inverseInertiaTensor = mat.Inverse();
    TransformInertiaTensor();
}

void RigidBody::SetInverseInertiaTensor(const Matrix3& mat)
{
    inverseInertiaTensor = mat;
    TransformInertiaTensor();
}

void RigidBody::SetPosition(const Vector& vec)
{
    position = vec;
    UpdateModelToWorldMatrix();
}

void RigidBody::SetPosition(float x, float y, float z)
{
    position.x = x;
    position.y = y;
    position.z = z;
    UpdateModelToWorldMatrix();
}

void physics::RigidBody::UpdatePosition(const Vector& vec){
    position += vec;
    UpdateModelToWorldMatrix();
}

void RigidBody::SetOrientation(const Quaternion& quat)
{
    orientation = quat;
    UpdateModelToWorldMatrix();
    TransformInertiaTensor();
}

void physics::RigidBody::SetOrientation(const Vector& vec, float angle)
{
    orientation = ConvertToQuaternion(vec,angle);
    UpdateModelToWorldMatrix();
    TransformInertiaTensor();
}

void physics::RigidBody::SetOrientation(Vector&& vec, float angle) noexcept
{
    orientation = ConvertToQuaternion(vec,angle);
    UpdateModelToWorldMatrix();
    TransformInertiaTensor();
}

void RigidBody::SetVelocity(const Vector& vec)
{
    velocity = vec;
}

void RigidBody::SetVelocity(float x, float y, float z)
{
    velocity.x = x;
    velocity.y = y;
    velocity.z = z;
}

void RigidBody::SetAngularVelocity(const Vector& vec)
{
    Matrix3 rotationMatrix;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            rotationMatrix.entries[3*i + j] = modelToWorldMat.entries[4*i + j];

    angularVelocity = rotationMatrix.Transpose() * vec;
}

void RigidBody::SetAngularVelocity(float x, float y, float z)
{
    Matrix3 rotationMatrix;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            rotationMatrix.entries[3*i + j] = modelToWorldMat.entries[4*i + j];
    
    Vector newRotation = rotationMatrix.Transpose() * Vector(x, y, z);

    angularVelocity = newRotation;
}

void RigidBody::SetAcceleration(const Vector& vec)
{
    acceleration = vec;
}

void RigidBody::SetAcceleration(float x, float y, float z)
{
    acceleration.x = x;
    acceleration.y = y;
    acceleration.z = z;
}


float RigidBody::GetMass() const
{
    return 1.0f / inverseMass;
}

float RigidBody::GetInverseMass() const
{
    return inverseMass;
}

Matrix3 RigidBody::GetInverseInertiaTensor() const
{
    return inverseInertiaTensor;
}

Matrix3 RigidBody::GetInverseInertiaTensorWorld() const
{
    return inverseInertiaTensorWorld;
}

Vector RigidBody::GetPosition() const
{
    return position;
}

Vector RigidBody::GetVelocity() const
{
    return velocity;
}

Vector RigidBody::GetAngularVelocity() const
{
    Matrix3 rotationMatrix;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            rotationMatrix.entries[3*i + j] = modelToWorldMat.entries[4*i + j];

    return rotationMatrix * angularVelocity;
}

Vector RigidBody::GetAcceleration() const
{
    return acceleration;
}

float RigidBody::GetLinearDamping() const
{
    return linearDamping;
}

void RigidBody::GetTransformMatrix(float matrix[16]) const
{
    matrix[0] = modelToWorldMat.entries[0];
    matrix[1] = modelToWorldMat.entries[4];
    matrix[2] = modelToWorldMat.entries[8];
    matrix[3] = modelToWorldMat.entries[12];

    matrix[4] = modelToWorldMat.entries[1];
    matrix[5] = modelToWorldMat.entries[5];
    matrix[6] = modelToWorldMat.entries[9];
    matrix[7] = modelToWorldMat.entries[13];

    matrix[8] = modelToWorldMat.entries[2];
    matrix[9] = modelToWorldMat.entries[6];
    matrix[10] = modelToWorldMat.entries[10];
    matrix[11] = modelToWorldMat.entries[14];

    matrix[12] = modelToWorldMat.entries[3];
    matrix[13] = modelToWorldMat.entries[7];
    matrix[14] = modelToWorldMat.entries[11];
    matrix[15] = modelToWorldMat.entries[15];
}

Matrix4 RigidBody::GetTransformMatrix() const
{
    return modelToWorldMat;
}

SphereRigidBody::SphereRigidBody(float _radius)
{
    radius = _radius;
}

void SphereRigidBody::UpdateVertex(double value, ...)
{
    radius = value;
}

void BoxRigidBody::UpdateVertex(double value, ...)
{
    scale.x = value;

    va_list args;
    va_start(args, value);

    scale.y = static_cast<float>(va_arg(args, double));
    scale.z = static_cast<float>(va_arg(args, double));

    va_end(args);
}
