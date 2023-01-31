#pragma once

#include <list>
#include "body.h"
#include "detector.h"
#include "resolver.h"
#include "renderer.h"
#include "../collisionSimulator/geometry.h"
#include "../collisionSimulator/contact_info.h"
#include "Vector.h"
#include <vector>
#include <unordered_map>
#include "../collisionSimulator/collisionSimulatorInfo.h"//all collisionSimulator specific things that will be passed to the game object manager
#include "../collisionSimulator/object.h"

using namespace Graphics;
using namespace physics;

class Camera;

class ObjectManager {
    friend class CollisionSimulator;
    friend class ObjectManager;

    typedef std::unordered_map<unsigned int, RigidBody*> RigidBodies;
    typedef std::vector<Contact> Contacts;
    typedef std::unordered_map<unsigned int, Object*> Objects;

    static constexpr int ImpulseIteration = 5;
public:
    ObjectManager() = default;
    ObjectManager(const ObjectManager&) = default;
    ObjectManager(ObjectManager&&)noexcept = default;
    ObjectManager& operator=(const ObjectManager&) = default;
    ObjectManager& operator=(ObjectManager&&) noexcept = default;
    ~ObjectManager();

    void SyncPlaygroundInfo(shared_ptr<CollisionSimulatorInfo> PlaygroundInfo) {
        collisionSimulatorSpecificInfo= PlaygroundInfo;
    }
    void Update(double dt);
    void Render(Renderer& renderer,const Graphics::Camera& camera);

    RigidBody* AddRigidBody(Geometry, const Vector& pos, const Vector& vel,float mass, const Vector& scale);
    unsigned int AddObject(Geometry, Renderer&, const Vector& pos = { 0.f,0.f,0.f }, const Vector& vel = { 0.f,0.f,0.f }, float mass = 5.f, const Vector& scale = {1.f,1.f,1.f});

    void GetContactInfo(std::vector<ContactInfo*>&) const;
private:
    Objects objects;

    shared_ptr<CollisionSimulatorInfo> collisionSimulatorSpecificInfo;
	std::vector<ContactInfo*> contactInfo;
    RigidBodies bodies;
    Contacts contacts;

    CollisionDetector detector;
    CollisionResolver resolver;
};