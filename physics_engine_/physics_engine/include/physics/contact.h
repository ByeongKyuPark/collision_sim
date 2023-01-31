#pragma once
#include "body.h"

namespace physics
{
    struct Contact
    {
        RigidBody* bodies[2];
        Vector normal;
        Vector* contactPoint[2];
        float penetration;
        float restitution;
        float friction;
        float normalImpulseSum;
        float tangentImpulseSum1;
        float tangentImpulseSum2;
    };
} 
