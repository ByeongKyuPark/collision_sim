#pragma once

#include "body.h"
#include "contact.h"
#include <vector>
#include <unordered_map>

class ObjectManager;

namespace physics
{

    class CollisionDetector
    {
        friend class ObjectManager;
        
    private:
        float friction;
        float objectRestitution;
        float groundRestitution;
    
    public:
        CollisionDetector()
            : friction(0.5f), objectRestitution(0.0005f), groundRestitution(0.0005f) {}
    
        void DetectCollision(
            std::vector<Contact>& contacts,
            std::unordered_map<unsigned int, RigidBody*>& colliders
        );
    
        void SetGroundRestitution(float value){
            groundRestitution = value;
        }
        void SetObjectRestitution(float value) {
            objectRestitution = value;
        }
    private:
        bool SphereAndBox(
            std::vector<Contact>& contacts,
            SphereRigidBody*,
            BoxRigidBody*
        );
        bool SphereAndSphere(
            std::vector<Contact>& contacts,
            SphereRigidBody*,
            SphereRigidBody*
        );
        bool SphereAndPlane(
            std::vector<Contact>& contacts,
            SphereRigidBody*,
            const Quaternion&orientation
        );
        bool BoxAndBox(
            std::vector<Contact>& contacts,
            BoxRigidBody*,
            BoxRigidBody*
        );
        bool BoxAndPlane(
            std::vector<Contact>& contacts,
            BoxRigidBody*,
            const Quaternion& orientation
        );
    
    private:
        float CalcPenetration(
            const BoxRigidBody* box1,
            const BoxRigidBody* box2,
            bool& shouldFlip,
            const Vector& axis
        );
        
        void CalcContactPointOnPlane(
            const BoxRigidBody* box1,
            const BoxRigidBody* box2,
            int minPenetrationAxisIdx,
            Contact& contact
        );

        void CalcContactPointOnLine(
            const BoxRigidBody* box1,
            const BoxRigidBody* box2,
            int minPenetrationAxisIdx,
            Contact& contact
        );
    };
}