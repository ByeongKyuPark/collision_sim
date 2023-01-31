#include <collisionSimulator/object.h>
#include <cstdarg>

void Object::getPositionInArray(float (&array)[3]) const
{
    physics::Vector position = body->GetPosition();
    array[0] = position.x;
    array[1] = position.y;
    array[2] = position.z;
}

void Object::getVelocityInArray(float (&array)[3]) const
{
    physics::Vector velocity = body->GetVelocity();
    array[0] = velocity.x;
    array[1] = velocity.y;
    array[2] = velocity.z;
}

void Object::getRotationInArray(float (&array)[3]) const
{
    physics::Vector rotation = body->GetAngularVelocity();
    array[0] = rotation.x;
    array[1] = rotation.y;
    array[2] = rotation.z;
}

void Object::getAccelerationInArray(float (&array)[3]) const
{
    physics::Vector acceleration = body->GetAcceleration();
    array[0] = acceleration.x;
    array[1] = acceleration.y;
    array[2] = acceleration.z;
}

void Object::getMassInArray(float (&array)[3]) const
{
    float mass = body->GetMass();
    array[0] = mass;
}

void Object::changeTexture(int idx){
    imageId = idx % (int)ImageID::NUM_IMAGES;
}

void SphereObject::getGeometricDataInArray(float (&array)[3]) const
{
    array[0] = radius;
}

void SphereObject::setGeometricData(double value, ...)
{
    radius = value;
}

void BoxObject::getGeometricDataInArray(float (&array)[3]) const
{
    array[0] = halfX;
    array[1] = halfY;
    array[2] = halfZ;
}

void BoxObject::setGeometricData(double value, ...)
{
    halfX = value;

    va_list args;
    va_start(args, value);
    
    halfY = va_arg(args, double);
    halfZ = va_arg(args, double);

    va_end(args);
}