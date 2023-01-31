#include <physics/detector.h>
#include <cmath>
#include <cfloat>
#include <typeinfo>
#include <array>
#include <iostream>

using namespace physics;
using std::array;

void CollisionDetector::DetectCollision(
    std::vector<Contact>& contacts,
    std::unordered_map<unsigned int, RigidBody*>& rigidBodies//,
    //PlaneRigidBody* groundRigidBody
)
{
    for (auto i = rigidBodies.begin(); i != rigidBodies.end(); ++i)
    {
        //RigidBody* colliderPtrI = i->second;
        for (auto j = i; j != rigidBodies.end(); ++j)
        {
            if (i == j) {
                continue;
            }
            //RigidBody* colliderPtrJ = j->second;
            //if (typeid(colliderPtrI) == typeid(SphereRigidBody))
            if (i->second->GetType()==BodyType::SPHERE)
            {
                //SphereRigidBody* collider1 = static_cast<SphereRigidBody*>(colliderPtrI);
                //if (typeid(colliderPtrJ) == typeid(SphereRigidBody)) // 구 - 구 충돌
                if(j->second->GetType()==BodyType::SPHERE)
                {
                    //SphereRigidBody* collider2 = static_cast<SphereRigidBody*>(colliderPtrJ);
                    //SphereAndSphere(contacts, collider1, collider2);
                    SphereAndSphere(contacts, dynamic_cast<SphereRigidBody*>(i->second), dynamic_cast<SphereRigidBody*>(j->second));
                }
                //else if (typeid(colliderPtrJ) == typeid(BoxRigidBody)) // 구 - 직육면체 충돌
                else if (j->second->GetType() == BodyType::BOX)
                {
                    //BoxRigidBody* collider2 = static_cast<BoxRigidBody*>(colliderPtrJ);
                    //SphereAndBox(contacts, collider1, collider2);
                    SphereAndBox(contacts, dynamic_cast<SphereRigidBody*>(i->second), dynamic_cast<BoxRigidBody*>(j->second));
                }
            }
            //else if (typeid(colliderPtrI) == typeid(BoxRigidBody))
            else if (i->second->GetType() == BodyType::BOX)
            {
                //BoxRigidBody* collider1 = static_cast<BoxRigidBody*>(colliderPtrI);
                //if (typeid(colliderPtrJ) == typeid(SphereRigidBody)) // 구 - 직육면체 충돌
                if (j->second->GetType()==BodyType::SPHERE)
                {
                    //SphereRigidBody* collider2 = static_cast<SphereRigidBody*>(colliderPtrJ);
                    //SphereAndBox(contacts, collider2, collider1);
                    SphereAndBox(contacts, dynamic_cast<SphereRigidBody*>(j->second), dynamic_cast<BoxRigidBody*>(i->second));
                }
                //else if (typeid(colliderPtrJ) == typeid(BoxRigidBody)) // 직육면체 - 직육면체 충돌
                else if (j->second->GetType()==BodyType::BOX)
                {
                    //BoxRigidBody* collider2 = static_cast<BoxRigidBody*>(colliderPtrJ);
                    //BoxAndBox(contacts, collider1, collider2);
                    if (i->second->IsFixed() == true) {
                        BoxAndPlane(contacts,dynamic_cast<BoxRigidBody*>(j->second),i->second->GetOrientation());
                    }
                    else if (j->second->IsFixed() == true) {
                        BoxAndPlane(contacts, dynamic_cast<BoxRigidBody*>(i->second), j->second->GetOrientation());
                    }
                    else {
                        BoxAndBox(contacts, dynamic_cast<BoxRigidBody*>(i->second), dynamic_cast<BoxRigidBody*>(j->second));
                    }
                }
            }
        }

        ///* 지면과의 충돌 검사 */
        ////if (typeid(colliderPtrI) == typeid(SphereRigidBody))
        //if (i->second->GetType() == BodyType::SPHERE)
        //{
        //    SphereRigidBody* sphereRigidBody = dynamic_cast<SphereRigidBody*>(i->second);
        //    SphereAndPlane(contacts, sphereRigidBody, groundRigidBody);
        //}
        ////else if (typeid(colliderPtrI) == typeid(BoxRigidBody))
        //else if (i->second->GetType() == BodyType::BOX)
        //{
        //    BoxRigidBody* boxRigidBody = dynamic_cast<BoxRigidBody*>(i->second);
        //    BoxAndPlane(contacts, boxRigidBody, groundRigidBody);
        //}
    }
}

bool CollisionDetector::SphereAndBox(
    std::vector<Contact>& contacts,
    SphereRigidBody* sphere,
    BoxRigidBody* box
)
{
    if (sphere == nullptr || box == nullptr) {
        return false;
    }
    /* 구의 중심을 직육면체의 로컬 좌표계로 변환한다 */
    Matrix4 worldToLocal = box->GetTransformMatrix().inverse();
    Vector sphereInBoxLocal = worldToLocal * sphere->GetPosition();

    /* 구의 중심과 가장 가까운 직육면체 위의 점을 찾는다 */
    Vector closestPoint;
    /* a 축 성분 비교 */
    if (sphereInBoxLocal.x > box->scale.x)
        closestPoint.x = box->scale.x;
    else if (sphereInBoxLocal.x < -box->scale.x)
        closestPoint.x = -box->scale.x;
    else
        closestPoint.x = sphereInBoxLocal.x;
    /* b 축 성분 비교 */
    if (sphereInBoxLocal.y > box->scale.y)
        closestPoint.y = box->scale.y;
    else if (sphereInBoxLocal.y < -box->scale.y)
        closestPoint.y = -box->scale.y;
    else
        closestPoint.y = sphereInBoxLocal.y;
    /* c 축 성분 비교 */
    if (sphereInBoxLocal.z > box->scale.z)
        closestPoint.z = box->scale.z;
    else if (sphereInBoxLocal.z < -box->scale.z)
        closestPoint.z = -box->scale.z;
    else
        closestPoint.z = sphereInBoxLocal.z;

    /* 위의 결과와 구의 중심 사이의 거리가
        구의 반지름보다 작다면 충돌이 발생한 것이다 */
    float distanceSquared = (closestPoint - sphereInBoxLocal).MagnitudeSq();
    if (distanceSquared < sphere->radius*sphere->radius)
    {
        /* 구에 가장 가까운 직육면체 위의 점을 월드 좌표계로 변환한다 */
        Vector closestPointWorld = box->GetTransformMatrix() * closestPoint;

        /* 충돌 정보를 생성한다 */
        Contact newContact{};
        newContact.bodies[0] = sphere;
        newContact.bodies[1] = box;
        newContact.normal = sphere->GetPosition() - closestPointWorld;
        newContact.normal.normalize();
        newContact.contactPoint[0] =
            new Vector(sphere->GetPosition() - newContact.normal * sphere->radius);
        newContact.contactPoint[1] = new Vector(closestPointWorld);
        newContact.penetration = sphere->radius - sqrtf(distanceSquared);
        newContact.restitution = objectRestitution;
        newContact.friction = friction;
        newContact.normalImpulseSum = 0.0f;
        newContact.tangentImpulseSum1 = 0.0f;
        newContact.tangentImpulseSum2 = 0.0f;

        contacts.push_back(newContact);
        return true;
    }
    else
        return false;
}

bool CollisionDetector::SphereAndSphere(
    std::vector<Contact>& contacts,
    SphereRigidBody* sphere1,
    SphereRigidBody* sphere2
)

{
    if(sphere1==nullptr || sphere2==nullptr){
        return false;
    }
    const Vector& lhsPos = sphere1->GetPosition();
    const Vector& rhsPos = sphere2->GetPosition();

    /* 두 구 사이의 거리를 구한다 */
    //float distanceSq =
    //    (sphere1->GetPosition() - sphere2->GetPosition()).MagnitudeSq();
    Vector midline = lhsPos - rhsPos;
    float distanceSq =midline.MagnitudeSq();
    /* 두 구 사이의 거리가 두 구의 반지름의 합보다 작다면 충돌이 발생한 것이다 */
    float radiusSum = sphere1->radius + sphere2->radius;

    if (distanceSq>0 && distanceSq < radiusSum*radiusSum)
    {
        midline.normalize();
        
        /* 충돌 정보를 생성한다 */
        Contact newContact{};
        newContact.bodies[0] = sphere1;
        newContact.bodies[1] = sphere2;
        newContact.normal = midline;
        newContact.contactPoint[0] =
            new Vector(sphere1->GetPosition() - midline * sphere1->radius);
        newContact.contactPoint[1] =
            new Vector(sphere2->GetPosition() + midline * sphere2->radius);
        newContact.penetration = radiusSum - sqrtf(distanceSq);
        newContact.restitution = objectRestitution;
        newContact.friction = friction;
        newContact.normalImpulseSum = 0.0f;
        newContact.tangentImpulseSum1 = 0.0f;
        newContact.tangentImpulseSum2 = 0.0f;

        contacts.push_back(newContact);
        return true;
    }
    else
        return false;
}

bool CollisionDetector::SphereAndPlane(
    std::vector<Contact>& contacts,
    SphereRigidBody* sphere,
    const Quaternion& orientation
    //PlaneRigidBody* plane
)
{
    if (sphere == nullptr){ //|| plane == nullptr) {
        return false;
    }
    /* 평면의 법선에 대한 구와 평면의 거리를 구한다 */
    const Vector normal = { orientation.a,orientation.b,orientation.c };
    float distance = normal.dot(sphere->GetPosition());
    distance -= orientation.d;

    /* 위 값이 구의 반지름보다 작다면 충돌이 발생한 것이다 */
    if (distance < sphere->radius)
    {
        /* 충돌 정보를 생성한다 */
        Contact newContact{};
        newContact.bodies[0] = sphere;
        newContact.bodies[1] = nullptr;
        newContact.normal = normal;
        newContact.contactPoint[0] = new Vector(sphere->GetPosition() - normal * distance);
        newContact.contactPoint[1] = nullptr;
        newContact.penetration = sphere->radius - distance;
        newContact.restitution = groundRestitution;
        newContact.friction = friction;
        newContact.normalImpulseSum = 0.0f;
        newContact.tangentImpulseSum1 = 0.0f;
        newContact.tangentImpulseSum2 = 0.0f;

        contacts.push_back(newContact);
        return true;
    }
    else
        return false;
}

bool CollisionDetector::BoxAndBox(
    std::vector<Contact>& contacts,
    BoxRigidBody* box1,
    BoxRigidBody* box2
)
{
    if (box1 == nullptr || box2 == nullptr) {
        return false;
    }
    //std::vector<Vector> testAxes;
    static constexpr int NumTestAxes = 15;
    std::array<Vector, NumTestAxes> testAxes;
	//const float* mat1Entries = box1->modelToWorldMat.entries;
 //   const float* mat2Entries = box2->modelToWorldMat.entries;
 //   std::array<Vector, NumTestAxes> testAxes = {
 //       Vector{mat1Entries[0],mat1Entries[4],mat1Entries[8]},
 //       Vector{mat1Entries[1],mat1Entries[5],mat1Entries[9]},
 //       Vector{mat1Entries[2],mat1Entries[6],mat1Entries[10]},
 //       Vector{mat2Entries[0],mat2Entries[4],mat2Entries[8]},
 //       Vector{mat2Entries[1],mat2Entries[5],mat2Entries[9]},
 //       Vector{mat2Entries[2],mat2Entries[6],mat2Entries[10]}
	//};


	////Normalize all axes
	//for (int i{}; i < NumTestAxes; ++i) {
	//	testAxes[i].Normalize();
	//}

    /* 겹침 검사의 기준이 될 축 저장 */
    //std::vector<Vector> axes;

    /* box1 의 세 축 저장 */
    /* box2 의 세 축 저장 */
    for (int i = 0; i < 3; ++i) {
        testAxes[i]= box1->GetAxis(i);
        testAxes[3+i] = box2->GetAxis(i);
    }

    //for (int i = 3; i < 6; ++i)
    //    testAxes.push_back(box2->GetAxis(i));

    //각 축 사이의 벡터외적
    //for (int i{}; i < 3; ++i) {
    //    testAxes[6 + i * 3 + 0] = testAxes[i].cross(testAxes[3]);
    //    testAxes[6 + i * 3 + 1] = testAxes[i].cross(testAxes[4]);
    //    testAxes[6 + i * 3 + 2] = testAxes[i].cross(testAxes[5]);
    //}

    /* 각 축 사이의 외적 저장 */
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            Vector crossProduct = testAxes[i].cross(testAxes[3 + j]);
            crossProduct.normalize();
            testAxes[6+i*3+j]=(crossProduct);
        }
    }

    float minPenetration = FLT_MAX;
    int minAxisIdx = 0;
    bool shouldFlip{false};

    /* 모든 축에 대해 겹침 검사 */
    for (int i = 0; i < NumTestAxes; ++i)
    {
        bool shouldFlipTemp{ false };
        float penetration = CalcPenetration(box1, box2, shouldFlipTemp,testAxes[i]);

        /* 한 축이라도 겹치지 않으면 충돌이 발생하지 않은 것이다 */
        if (penetration <= 0.0f)
            return false;

        /* 가장 적게 겹치는 정도와 그때의 기준 축을 추적한다 */
        if (penetration <= minPenetration)
        {
            shouldFlip = shouldFlipTemp;
            minPenetration = penetration;
            minAxisIdx = i;
        }
    }

    /* 모든 축에 걸쳐 겹침이 감지됐다면 충돌이 발생한 것이다 */
    Contact newContact{};
    newContact.bodies[0] = box1;
    newContact.bodies[1] = box2;
    newContact.penetration = minPenetration;
    newContact.restitution = objectRestitution;
    newContact.friction = friction;
    newContact.normalImpulseSum = 0.0f;
    newContact.tangentImpulseSum1 = 0.0f;
    newContact.tangentImpulseSum2 = 0.0f;
    if (shouldFlip == true) {
        newContact.normal = -testAxes[minAxisIdx];
    }
    else {
        newContact.normal = testAxes[minAxisIdx];
    }

    if (minAxisIdx < 6)
    {
        CalcContactPointOnPlane(box1, box2, minAxisIdx, newContact);
    }
    else
    {
        CalcContactPointOnLine(box1, box2, minAxisIdx, newContact);
    }
    
    contacts.push_back(newContact);
    return true;
}

bool CollisionDetector::BoxAndPlane(
    std::vector<Contact>& contacts,
    BoxRigidBody* box,
    const Quaternion& orientation
)
{
    if (box == nullptr) {
        return false;
    }

    Vector vertices[8];
    vertices[0] = Vector(-box->scale.x, box->scale.y, box->scale.z);
    vertices[1] = Vector(-box->scale.x, -box->scale.y, box->scale.z);
    vertices[2] = Vector(box->scale.x, -box->scale.y, box->scale.z);
    vertices[3] = Vector(box->scale.x, box->scale.y, box->scale.z);

    vertices[4] = Vector(-box->scale.x, box->scale.y, -box->scale.z);
    vertices[5] = Vector(-box->scale.x, -box->scale.y, -box->scale.z);
    vertices[6] = Vector(box->scale.x, -box->scale.y, -box->scale.z);
    vertices[7] = Vector(box->scale.x, box->scale.y, -box->scale.z);

    for (int i = 0; i < 8; ++i) {
        vertices[i] = box->GetTransformMatrix() * vertices[i];
    }

    bool hasContacted = false;
    const Vector normal = { orientation.a,orientation.b,orientation.c };

    for (int i = 0; i < 8; ++i)
    {
        float distance = normal.dot(vertices[i]);
        distance += orientation.d;

        if (distance < 0)
        {
            Contact newContact{};
            newContact.bodies[0] = box;
            newContact.bodies[1] = nullptr;
            newContact.normal = normal;
            newContact.contactPoint[0] = new Vector(vertices[i]);
            newContact.contactPoint[1] = nullptr;
            newContact.penetration = -distance;
            newContact.restitution = groundRestitution;
            newContact.friction = friction;
            newContact.normalImpulseSum = 0.0f;
            newContact.tangentImpulseSum1 = 0.0f;
            newContact.tangentImpulseSum2 = 0.0f;

            contacts.push_back(newContact);
            hasContacted = true;
        }
    }

    return hasContacted;
}


float CollisionDetector::CalcPenetration(const BoxRigidBody* box1, const BoxRigidBody* box2,bool& shouldFlip, const Vector& testingAxis){
    Vector centerToCenter = box2->GetPosition() - box1->GetPosition();
    float projectedCenterToCenter = abs(centerToCenter.dot(testingAxis));
    const Vector box1AxisX = box1->GetAxis(0);
    const Vector box1AxisY = box1->GetAxis(1);
    const Vector box1AxisZ = box1->GetAxis(2);

    const Vector box2AxisX = box2->GetAxis(0);
    const Vector box2AxisY = box2->GetAxis(1);
    const Vector box2AxisZ = box2->GetAxis(2);

    float projectedSum =
        abs((box1AxisX * box1->scale.x).dot(testingAxis))
        + abs((box1AxisY * box1->scale.y).dot(testingAxis))
        + abs((box1AxisZ * box1->scale.z).dot(testingAxis))
        + abs((box2AxisX * box2->scale.x).dot(testingAxis))
        + abs((box2AxisY * box2->scale.y).dot(testingAxis))
        + abs((box2AxisZ * box2->scale.z).dot(testingAxis));

    if (testingAxis.dot(centerToCenter) > 0) {
        shouldFlip = true;
    }

    return projectedSum - projectedCenterToCenter;
}

void CollisionDetector::CalcContactPointOnPlane(
    const BoxRigidBody* box1,
    const BoxRigidBody* box2,
    int minAxisIdx,
    Contact& contact
)
{
    Vector* contactPoint1;
    Vector* contactPoint2;

    if (minAxisIdx < 3)
    {
        const Vector box2AxisX = box2->GetAxis(0);
        const Vector box2AxisY = box2->GetAxis(1);
        const Vector box2AxisZ = box2->GetAxis(2);

        contactPoint2 = new Vector(box2->GetPosition());
        if (contact.normal.dot(box2AxisX) < 0) {
            *contactPoint2 -= box2AxisX * box2->scale.x;
        }
        else {
            *contactPoint2 += box2AxisX * box2->scale.x;
        }
        if (contact.normal.dot(box2AxisY) < 0) {
            *contactPoint2 -= box2AxisY * box2->scale.y;
        }
        else {
            *contactPoint2 += box2AxisY * box2->scale.y;
        }
        if (contact.normal.dot(box2AxisZ) < 0) {
            *contactPoint2 -= box2AxisZ * box2->scale.z;
        }
        else {
            *contactPoint2 += box2AxisZ * box2->scale.z;
        }

        contactPoint1 = new Vector(*contactPoint2 - contact.normal * contact.penetration);
    }
    else
    {
        contactPoint1 = new Vector(box1->GetPosition());

        const Vector box1AxisX = box1->GetAxis(0);
        const Vector box1AxisY = box1->GetAxis(1);
        const Vector box1AxisZ = box1->GetAxis(2);

        contactPoint1 = new Vector(box1->GetPosition());
        if (contact.normal.dot(box1AxisX) > 0) {
            *contactPoint1 -= box1AxisX * box1->scale.x;
        }
        else {
            *contactPoint1 += box1AxisX * box1->scale.x;
        }
        if (contact.normal.dot(box1AxisY) > 0) {
            *contactPoint1 -= box1AxisY * box1->scale.y;
        }
        else {
            *contactPoint1 += box1AxisY * box1->scale.y;
        }
        if (contact.normal.dot(box1AxisZ) > 0) {
            *contactPoint1 -= box1AxisZ * box1->scale.z;
        }
        else {
            *contactPoint1 += box1AxisZ * box1->scale.z;
        }

        contactPoint2 = new Vector(*contactPoint1 - contact.normal * contact.penetration);
    }

    contact.contactPoint[0] = contactPoint1;
    contact.contactPoint[1] = contactPoint2;
}

void CollisionDetector::CalcContactPointOnLine(
    const BoxRigidBody* box1,
    const BoxRigidBody* box2,
    int minAxisIdx,
    Contact& contact
)
{

    const Vector box1AxisX = box1->GetAxis(0);
    const Vector box1AxisY = box1->GetAxis(1);
    const Vector box1AxisZ = box1->GetAxis(2);
    bool isPositiveX1 = false;
    bool isPositiveY1 = false;
    bool isPositiveZ1 = false;

    Vector vertexOne{ box1->GetPosition() };
    if (contact.normal.dot(box1AxisX) > 0) {
        vertexOne -= box1AxisX * box1->scale.x;
    }
    else {
        vertexOne += box1AxisX * box1->scale.x;
        isPositiveX1 = true;
    }
    if (contact.normal.dot(box1AxisY) > 0) {
        vertexOne -= box1AxisY * box1->scale.y;
    }
    else {
        vertexOne += box1AxisY * box1->scale.y;
        isPositiveY1 = true;
    }
    if (contact.normal.dot(box1AxisZ) > 0) {
        vertexOne -= box1AxisZ * box1->scale.z;
    }
    else {
        vertexOne += box1AxisZ * box1->scale.z;
        isPositiveZ1 = true;
    }
    const Vector box2AxisX = box2->GetAxis(0);
    const Vector box2AxisY = box2->GetAxis(1);
    const Vector box2AxisZ = box2->GetAxis(2);
    bool isPositiveX2 = false;
    bool isPositiveY2 = false;
    bool isPositiveZ2 = false;

    Vector vertexTwo{ box2->GetPosition() };
    if (contact.normal.dot(box2AxisX) < 0) {
        vertexTwo -= box2AxisX * box2->scale.x;
    }
    else {
        vertexTwo += box2AxisX * box2->scale.x;
        isPositiveX2 = true;
    }
    if (contact.normal.dot(box2AxisY) < 0) {
        vertexTwo -= box2AxisY * box2->scale.y;
    }
    else {
        vertexTwo += box2AxisY * box2->scale.y;
        isPositiveY2 = true;
    }
    if (contact.normal.dot(box2AxisZ) < 0) {
        vertexTwo -= box2AxisZ * box2->scale.z;
    }
    else {
        vertexTwo += box2AxisZ * box2->scale.z;
        isPositiveZ2 = true;
    }

    Vector directionBox1, directionBox2;
    
    switch (minAxisIdx)
    {
    case 6:
        directionBox1 = (isPositiveX1 < 0) ? box1->GetAxis(0) : -box1->GetAxis(0);
        directionBox2 = (isPositiveX2 < 0) ? box2->GetAxis(0) : -box2->GetAxis(0);
        break;

    case 7:
        directionBox1 = (isPositiveX1 < 0) ? box1->GetAxis(0) : -box1->GetAxis(0);
        directionBox2 = (isPositiveY2 < 0) ? box2->GetAxis(1) : -box2->GetAxis(1);
        break;

    case 8:
        directionBox1 = (isPositiveX1 < 0) ? box1->GetAxis(0) : -box1->GetAxis(0);
        directionBox2 = (isPositiveZ2 < 0) ? box2->GetAxis(2) : -box2->GetAxis(2);
        break;

    case 9:
        directionBox1 = (isPositiveY1 < 0) ? box1->GetAxis(1) : -box1->GetAxis(1);
        directionBox2 = (isPositiveX2 < 0) ? box2->GetAxis(0) : -box2->GetAxis(0);
        break;

    case 10:
        directionBox1 = (isPositiveY1 < 0) ? box1->GetAxis(1) : -box1->GetAxis(1);
        directionBox2 = (isPositiveY2 < 0) ? box2->GetAxis(1) : -box2->GetAxis(1);
        break;
    
    case 11:
        directionBox1 = (isPositiveY1 < 0) ? box1->GetAxis(1) : -box1->GetAxis(1);
        directionBox2 = (isPositiveZ2 < 0) ? box2->GetAxis(2) : -box2->GetAxis(2);
        break;
    
    case 12:
        directionBox1 = (isPositiveZ1 < 0) ? box1->GetAxis(2) : -box1->GetAxis(2);
        directionBox2 = (isPositiveX2 < 0) ? box2->GetAxis(0) : -box2->GetAxis(0);
        break;
    
    case 13:
        directionBox1 = (isPositiveZ1 < 0) ? box1->GetAxis(2) : -box1->GetAxis(2);
        directionBox2 = (isPositiveY2 < 0) ? box2->GetAxis(1) : -box2->GetAxis(1);
        break;
    
    case 14:
        directionBox1 = (isPositiveZ1 < 0) ? box1->GetAxis(2) : -box1->GetAxis(2);
        directionBox2 = (isPositiveZ2 < 0) ? box2->GetAxis(2) : -box2->GetAxis(2);
        break;
    
    default:
        break;
    }

    float k = directionBox1.dot(directionBox2);
    Vector* closestPointOne =
        new Vector(vertexOne + directionBox1 * ((vertexTwo-vertexOne).dot(directionBox1-directionBox2*k)/(1-k*k)));
    
    Vector* closestPointTwo =
        new Vector(vertexTwo + directionBox2 * ((*closestPointOne-vertexTwo).dot(directionBox2)));

    contact.contactPoint[0] = closestPointOne;
    contact.contactPoint[1] = closestPointTwo;
}
