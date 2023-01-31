#pragma once

#include "Vector.h"
#include "matrix3.h"
#include "matrix4.h"
#include "quaternion.h"

namespace physics
{
    enum class BodyType {NONE,SPHERE,BOX,PLANE,NUM_TYPES};

    class RigidBody
    {
    protected:        
        Vector force;
        Vector torque;
        Vector position;
        Vector velocity;
        Vector acceleration;
        Vector angularVelocity;

        Quaternion orientation;
        
        float inverseMass;
        float linearDamping;
        float angularDamping;

        Matrix3 inverseInertiaTensor;    
        Matrix3 inverseInertiaTensorWorld; 
        Matrix4 modelToWorldMat;

    public:
        //Rule of 5
        RigidBody() : isAwake{ true }, linearDamping(1.0f), angularDamping(0.35f), inverseMass{}, body{} {}
        RigidBody(const RigidBody&) = default;
        RigidBody(RigidBody&&) noexcept = default;
        RigidBody& operator=(const RigidBody&) = default;
        RigidBody& operator=(RigidBody&&) noexcept = default;
        virtual ~RigidBody() {}

        virtual void UpdateVertex(double, ...) = 0;
        virtual BodyType GetType()const = 0;

        void Update(float duration);
        void ApplyForce();
        Vector GetAxis(int index) const;
        void Rotate(const Quaternion&);

        bool IsFixed() {return inverseMass == 0.0f ? true : false;}

    private:    
        void UpdateModelToWorldMatrix();
        void QuaternionToRotateMatrix();
        void TransformInertiaTensor();

        BodyType body;
        bool isAwake;

    public:
        void Awake(bool awake) { isAwake=awake; }

        void SetMass(float value);
        void SetInverseMass(float value);

        void SetInertiaTensor(const Matrix3& mat);
        void SetInverseInertiaTensor(const Matrix3& mat);

        void SetPosition(const Vector& vec);
        void SetPosition(float x, float y, float z);
        void UpdatePosition(const Vector& vec);

        void SetOrientation(const Quaternion&);
        void SetOrientation(const Vector& vec,float angle);
        void SetOrientation(Vector&& vec, float angle) noexcept;

        void SetVelocity(const Vector& vec);
        void SetVelocity(float x, float y, float z);

        void SetAngularVelocity(const Vector& vec);
        void SetAngularVelocity(float x, float y, float z);

        void SetAcceleration(const Vector& vec);
        void SetAcceleration(float x, float y, float z);

        Vector GetPosition() const;
        Vector GetVelocity() const;
        Vector GetAngularVelocity() const;
        Vector GetAcceleration() const;
        Quaternion GetOrientation() const { return orientation; }

        bool IsAwake() const { return isAwake; }
        float GetMass() const;
        float GetInverseMass() const;
        Matrix3 GetInverseInertiaTensor() const;
        Matrix3 GetInverseInertiaTensorWorld() const;
        float GetLinearDamping() const;

        void GetTransformMatrix(float matrix[16]) const;
        Matrix4 GetTransformMatrix() const;
    }; 

    class SphereRigidBody : public RigidBody
    {
        friend class CollisionDetector;
        friend class Simulator;

    protected:
        float radius;

    public:
        SphereRigidBody(float _radius);
        void UpdateVertex(double, ...);
        BodyType GetType()const override final {
            return BodyType::SPHERE;
        }
    };

    class BoxRigidBody : public RigidBody
    {
        friend class CollisionDetector;
        friend class Simulator;

    protected:
        Vector scale;

    public:
        BoxRigidBody(const Vector& scl) :scale{ scl } {}
        void UpdateVertex(double, ...);
        BodyType GetType()const override final{
            return BodyType::BOX;
        }
    };
}
