#pragma once

#include "contact.h"
#include <vector>

namespace physics
{
    class CollisionResolver
    {
    private:
        int iterationLimit;
        float penetrationTolerance;
        float closingSpeedTolerance;

        friend class ObjectManager;
           
    public:
        CollisionResolver()
            : iterationLimit(30), penetrationTolerance(0.0005f), closingSpeedTolerance(0.005f) {}
    
    private:
        void ApplyImpulse(Contact, float deltaTime);
    };
} 