#include <physics/resolver.h>
#include <cmath>
#include <iostream>
#include <collisionSimulator/collisionSimulatorInfo.h>

using namespace physics;

void CollisionResolver::ApplyImpulse(Contact contact, float deltaTime)
{
    //Linear impulse
    float invMass1 = contact.bodies[0]->GetInverseMass();
    float invMass2 = (contact.bodies[1]==nullptr) ? 0.f: contact.bodies[1]->GetInverseMass();
    float invMassSum = invMass1 + invMass2;

    //collision between infinite masses
    if (invMassSum == 0.0f) {
        return;//nothing to calculate
    }

    Vector r1 = *contact.contactPoint[0] - contact.bodies[0]->GetPosition();
    Vector r2;
    if (contact.bodies[1] != nullptr)
        r2 = *contact.contactPoint[1] - contact.bodies[1]->GetPosition();
    Matrix3 i1 = contact.bodies[0]->GetInverseInertiaTensorWorld();
    Matrix3 i2 = (contact.bodies[1] == nullptr) ? Matrix3{}:contact.bodies[1]->GetInverseInertiaTensorWorld();

    Vector velocity1 = contact.bodies[0]->GetVelocity();
    Vector velocity2 = (contact.bodies[1] == nullptr) ? Vector{}:contact.bodies[1]->GetVelocity();
    Vector angularVelocity1 = contact.bodies[0]->GetAngularVelocity();
    Vector angularVelocity2 = (contact.bodies[1] == nullptr) ? Vector{} : contact.bodies[1]->GetAngularVelocity();

    Vector relativeVel = (velocity2 + angularVelocity2.cross(r2)) - (velocity1 + angularVelocity1.cross(r1));
    // Relative collision normal
    Vector relativeNorm = contact.normal;
    relativeNorm.normalize();

    //skip if both objects are moving away each other.
    if (relativeVel.dot(relativeNorm) < 0.f) {
        return;
    }
    float e = contact.restitution;

    float numerator = (-(1.0f + e) * relativeVel.dot(relativeNorm));
    float d1 = invMassSum;

    Vector d2 = (i1 * r1.cross(relativeNorm)).cross(r1);
    Vector d3 = (i2 * r2.cross(relativeNorm)).cross(r2);
    float denominator = d1 + relativeNorm.dot(d2 + d3);

    float j = (denominator == 0.0f) ? 0.0f : numerator / denominator;

    Vector impulse = relativeNorm * j;
    velocity1 -= impulse * invMass1;
    velocity2 += impulse * invMass2;


    angularVelocity1 -= i1 * r1.cross(impulse);
    angularVelocity2 += i2 * r2.cross(impulse);

    // Friction
    Vector t = relativeVel - (relativeNorm * relativeVel.dot(relativeNorm));
    if (t.MagnitudeSq()==0.f) {
        return;
    }
    t.normalize();

    numerator = -relativeVel.dot(t);
    d1 = invMassSum;
    d2 = (i1*r1.cross(t)).cross(r1);
    d3 = (i2 * r2.cross(t)).cross(r2);
    denominator = d1 + t.dot(d2 + d3);

    float jt = (denominator == 0.0f) ? 0.0f : numerator / denominator;

    if (jt==0.f) {
        return;
    }
    Vector tangentImpuse;
    float friction = contact.friction;

    tangentImpuse = t * jt;

    velocity1 -= tangentImpuse * invMass1;
    velocity2 += tangentImpuse * invMass2;

    angularVelocity1 -= i1 *(r1.cross(tangentImpuse));
    angularVelocity2 += i2 *(r2.cross(tangentImpuse));

    if (velocity1.MagnitudeSq() < Idle_Threshold&& angularVelocity1.MagnitudeSq()< Idle_Threshold) {
        contact.bodies[0]->Awake(false);
    }
    else {
        if (contact.bodies[0]->IsAwake() == false) {
            contact.bodies[0]->Awake(true);
        }
        contact.bodies[0]->SetVelocity(velocity1);
	    contact.bodies[0]->SetAngularVelocity(angularVelocity1);
    }

    if (contact.bodies[1] != nullptr) {
        if (velocity2.MagnitudeSq() < Idle_Threshold && angularVelocity2.MagnitudeSq() < Idle_Threshold) {
            contact.bodies[1]->Awake(false);
        }
        else{
            if (contact.bodies[1]->IsAwake() == false) {
                contact.bodies[1]->Awake(true);
            }
            contact.bodies[1]->SetVelocity(velocity2);
		    contact.bodies[1]->SetAngularVelocity(angularVelocity2);
        }
    }
}
