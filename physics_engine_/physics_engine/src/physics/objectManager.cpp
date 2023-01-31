#include <iterator>
#include <typeinfo>
#include <cmath>
#include <iostream>
#include <physics/objectManager.h>
#include <cstdarg>

using namespace physics;

extern float g_Gravity = -9.7;

ObjectManager::~ObjectManager(){
	for (auto& body : bodies) {
		delete body.second;
		body.second = nullptr;
	}
}

void ObjectManager::Update(double dt) {
	dt *= collisionSimulatorSpecificInfo->deltaTimeMultiplier;

	detector.DetectCollision(contacts, bodies);

	/* apply force */
	for (auto& body : bodies){
		if (body.second->IsAwake() == true) {
			body.second->ApplyForce();
		}
	}

	for (int i = 0; i < ImpulseIteration; ++i) {
		for (auto& contact : contacts) {
			//debug
			resolver.ApplyImpulse(contact, dt);
		}
	}
	/* Update */
	for (auto& body : bodies)
	{
		if (body.second->IsAwake() == true) {
			body.second->Update(dt);
		}

	}
	// Correct position to avoid sinking!
	for (const Contact& contact : contacts) {
		float inverseMass1 = contact.bodies[0]->GetInverseMass();
		float inverseMass2 = (contact.bodies[1] == nullptr) ? 0 : contact.bodies[1]->GetInverseMass();
		float totalMass = inverseMass1 + inverseMass2;

		if (totalMass == 0.0f) {//skip collision between infinite masses
			continue;
		}
		static constexpr float Density = -0.05f;
		float depth = fmaxf(contact.penetration - resolver.penetrationTolerance, 0.0f);
		float scalar = (totalMass == 0.0f) ? 0.0f : depth / totalMass;
		Vector correction = contact.normal * scalar * Density;//resolver.linearProjectionPercent;
		//https://gamedev.stackexchange.com/questions/114728/how-to-stop-sinking-bodies-in-physics-engine-c

		contact.bodies[0]->UpdatePosition(correction * -inverseMass1);
		if (contact.bodies[1] != nullptr) {
			contact.bodies[1]->UpdatePosition(correction * inverseMass2);
		}
	}
	contacts.clear();
}

void ObjectManager::Render(Renderer& renderer, const Graphics::Camera& camera) {
	renderer.UpdateWindowSize();

	glClearColor(0.15f, 0.15f, 0.1f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/* 배경 렌더 */
	renderer.RenderBackground(camera);
	/* 오브젝트 렌더 */
	for (auto& object : objects)
	{
		float modelMatrix[16];
		object.second->body->GetTransformMatrix(modelMatrix);
		renderer.RenderObject(
			object.second->geometry,
			object.second->id,
			object.second->imageId,
			modelMatrix,
			camera
		);
	}
	contactInfo.clear();
}


RigidBody* ObjectManager::AddRigidBody(Geometry geometry, const Vector& pos, const Vector& vel, float mass, const Vector& scale) {
	RigidBody* newBody{};

	/* 강체의 관성 모멘트 텐서를 도형에 따라 결정한다 */
	Matrix3 inertiaTensor;
	if (geometry == SPHERE)
	{
		newBody = new SphereRigidBody(scale.x);
		newBody->SetMass(mass);
		newBody->SetPosition(pos);
		newBody->SetVelocity(vel);

		float value = 0.4f * newBody->GetMass();
		inertiaTensor.SetDiagonal(value);
	}
	else if (geometry == BOX)
	{
		newBody = new BoxRigidBody(scale);
		newBody->SetMass(mass);
		newBody->SetPosition(pos);
		newBody->SetVelocity(vel);

		float value = newBody->GetMass() / 6.0f;
		inertiaTensor.SetDiagonal(value);
	}
	newBody->SetAcceleration(0.0f, g_Gravity, 0.0f);
	newBody->SetInertiaTensor(inertiaTensor);

	bodies[collisionSimulatorSpecificInfo->newObjectID] = newBody;

	return newBody;
}

void ObjectManager::GetContactInfo(std::vector<ContactInfo*>& cntctInfo) const
{
	for (const auto& contact : contacts)
	{
		for (const auto& cp : contact.contactPoint)
		{
			if (cp == nullptr)
				continue;

			ContactInfo* newContactInfo = new ContactInfo;
			newContactInfo->pointX = cp->x;
			newContactInfo->pointY = cp->y;
			newContactInfo->pointZ = cp->z;
			newContactInfo->normalX = contact.normal.x;
			newContactInfo->normalY = contact.normal.y;
			newContactInfo->normalZ = contact.normal.z;
			cntctInfo.push_back(newContactInfo);
		}
	}
}

//void ObjectManager::setGravity(float value)
//{
//    gravity = value;
//    for (auto& body : bodies)
//        body.second->SetAcceleration(0.0f, -gravity, 0.0f);
//}

unsigned int ObjectManager::AddObject(Geometry geometry, Renderer& renderer, const Vector& pos, const Vector& vel, float mass, const Vector& scale)//floats to Vector
{
	//Vector scl(scale);

	//va_list args;
	//va_start(args, scale);
	//scl.y = va_arg(args, double);
	//scl.z = va_arg(args, double);
	//va_end(args);

	//scl.y = (scl.y == 0.f) ? scl.x : scl.y;
	//scl.z = (scl.z == 0.f) ? scl.x : scl.z;

	Object* newObject{ nullptr };

	/* 주어진 도형에 따라 인스턴스를 생성한다 */
	if (geometry == SPHERE)
	{
		newObject = new SphereObject(scale.x);
		newObject->geometry = SPHERE;
		//std::cout << "DEBUG::CollisionSimulator::add sphere object id: " << newObjectID << std::endl;
	}
	else if (geometry == BOX)
	{
		newObject = new BoxObject(scale);
		newObject->geometry = BOX;
		//std::cout << "DEBUG::CollisionSimulator::add box object id: " << newObjectID << std::endl;
	}
	else {
		std::cout << "unknown object\n";
		exit(1);
	}
	/* id 를 부여한다 */
	newObject->id = collisionSimulatorSpecificInfo->newObjectID;


	/* 강체와 충돌체를 추가한다 */
	newObject->body = AddRigidBody(geometry, pos, vel, mass, scale);

	/* Shape 을 추가한다 */
	newObject->shape = renderer.addShape(collisionSimulatorSpecificInfo->newObjectID, geometry, scale);

	objects[collisionSimulatorSpecificInfo->newObjectID] = newObject;
	return collisionSimulatorSpecificInfo->newObjectID++;
}
