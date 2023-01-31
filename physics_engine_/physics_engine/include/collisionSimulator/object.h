#pragma once

#include "graphics/shape.h"
#pragma once

#include "geometry.h"
#include "physics/body.h"
#include "graphics/opengl/glm/glm.hpp"
#include "graphics/imageID.h"

using namespace Graphics;

class CollisionSimulator;
class ObjectManager;

class Object
{
    friend class CollisionSimulator;
    friend class ObjectManager;

protected:
    unsigned int id;
    unsigned int imageId;
    Geometry geometry;
    physics::RigidBody* body;
    Graphics::Shape* shape;

public:
    //rule of 5 (due to virtual d'tor)
    Object(ImageID imgId) : id{}, imageId((unsigned)imgId), geometry{}, body{ nullptr }, shape{ nullptr } {}
    Object(const Object&) = default;
    Object(Object&&) = default;
    Object& operator=(const Object&) = default;
    Object& operator=(Object&&) = default;
    virtual ~Object() {}

    unsigned int getID() const { return id; }
    Geometry getGeometry() const { return geometry; }
    void getPositionInArray(float (&array)[3]) const;
    void getVelocityInArray(float (&array)[3]) const;
    void getRotationInArray(float (&array)[3]) const;
    void getAccelerationInArray(float (&array)[3]) const;
    void getMassInArray(float (&array)[3]) const;
    virtual void getGeometricDataInArray(float (&array)[3]) const = 0;

    void changeTexture(int idx);
    virtual void setGeometricData(double, ...) = 0;
};

class SphereObject : public Object
{
protected:
    float radius;

public:
    SphereObject(float Radius=1.f,ImageID imgID = ImageID::BRICKS) : radius(Radius), Object{imgID} {}

    void getGeometricDataInArray(float (&array)[3]) const;
    void setGeometricData(double, ...);
};

class BoxObject : public Object
{
protected:
    float halfX;
    float halfY;
    float halfZ;

public:
    BoxObject(const Vector& scl,ImageID imgId=ImageID::FACE) : halfX(scl.x), halfY(scl.y), halfZ(scl.z), Object(imgId) {}

    void getGeometricDataInArray(float (&array)[3]) const;
    void setGeometricData(double, ...);
};
