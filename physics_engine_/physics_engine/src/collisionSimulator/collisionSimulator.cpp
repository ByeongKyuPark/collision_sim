#include <collisionSimulator/collisionSimulator.h>
#include <typeinfo>
#include <cmath>
#include <physics/math.h>

CollisionSimulator::CollisionSimulator()
    : shouldPause{true}, collisionSimulatorSpecificInfo{ std::make_shared<CollisionSimulatorInfo>() }
{
    (*collisionSimulatorSpecificInfo).newObjectID = 1;
    (*collisionSimulatorSpecificInfo).shouldRenderContactInfo = false;
    (*collisionSimulatorSpecificInfo).deltaTimeMultiplier = 1.f;
    objectManager.SyncPlaygroundInfo(collisionSimulatorSpecificInfo);

    LoadDefaultScenario();
}

void CollisionSimulator::run()
{
    double prevTime = glfwGetTime();
    double currTime, dt;

    while (!glfwWindowShouldClose(renderer.GetWindow())){
        HandleInput();

        currTime = glfwGetTime();
        dt = currTime - prevTime;
        prevTime = currTime;

        if (shouldPause == false) {
            objectManager.Update(dt);
        }
        objectManager.Render(renderer,camera);

        //(need a way to communicate between the object manager &  UI)

        renderer.BindDefaultFrameBuffer();
        renderer.SetWindowViewport();
        
        glfwSwapBuffers(renderer.GetWindow());
        glfwPollEvents();
    }
}

void CollisionSimulator::HandleInput()
{
    if (glfwGetKey(renderer.GetWindow(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(renderer.GetWindow(), GLFW_TRUE);
    }
    
    if (glfwGetKey(renderer.GetWindow(), GLFW_KEY_D) == GLFW_PRESS) {
        camera.MoveLeft();
    }
    else if (glfwGetKey(renderer.GetWindow(), GLFW_KEY_A) == GLFW_PRESS) {
        camera.MoveRight();
    }
    if (glfwGetKey(renderer.GetWindow(), GLFW_KEY_W) == GLFW_PRESS) {
        camera.MoveForward();
    }
    else if (glfwGetKey(renderer.GetWindow(), GLFW_KEY_S) == GLFW_PRESS) {
        camera.MoveBack();
    }
    if (glfwGetKey(renderer.GetWindow(), GLFW_KEY_UP) == GLFW_PRESS) {
        camera.MoveUp();
    }
    else if (glfwGetKey(renderer.GetWindow(), GLFW_KEY_DOWN) == GLFW_PRESS) {
        camera.MoveDown();
    }

    static bool isSpacePressed = false;
    if (glfwGetKey(renderer.GetWindow(), GLFW_KEY_SPACE) == GLFW_PRESS){
        if (!isSpacePressed){
            shouldPause = !shouldPause;
            isSpacePressed = true;
        }
    }
    else if (glfwGetKey(renderer.GetWindow(), GLFW_KEY_SPACE) == GLFW_RELEASE) {
        isSpacePressed = false;
    }

    static bool isFRepeated = false;
    if (glfwGetKey(renderer.GetWindow(), GLFW_KEY_F) == GLFW_PRESS){//drawFramesOnly
        if (isFRepeated==false){
            renderer.drawFramesOnly = !renderer.drawFramesOnly;
            isFRepeated = true;
        }
    }
    else if (glfwGetKey(renderer.GetWindow(), GLFW_KEY_F) == GLFW_RELEASE) {
        isFRepeated = false;
    }}

void CollisionSimulator::LoadDefaultScenario()
{
    constexpr float LightMass = 5.f;
    constexpr float HeavyMass = 150.f;

    unsigned int id = objectManager.AddObject(SPHERE, renderer, { 0.0f, 1.0f, 7.0f }, { 0.f,-2.f,-20.f }, LightMass, {0.5f,0.5f,0.5f});

    id = objectManager.AddObject(BOX, renderer, { 0.0f, -10.0f, -5.0f }, { 0.f,0.f,0.f }, INFINITY, { 200.f, 10.f, 200.f });
    objectManager.objects.find(id)->second->changeTexture(2);
    objectManager.objects.find(id)->second->body->SetInverseInertiaTensor(physics::Matrix3(0.0f));

    id = objectManager.AddObject(SPHERE, renderer, { 0.0f, 1.0f, 14.0f }, { 0.f,0.f,-20.f }, LightMass, { 0.75f,0.75f,0.75f });
    id = objectManager.AddObject(SPHERE, renderer, { 0.0f, 1.0f, 21.0f }, {0.f,0.f,-20.f});
    objectManager.objects.find(id)->second->body->SetVelocity(0.0f, 0.0f, -20.0f);

    id = objectManager.AddObject(SPHERE, renderer, { 0.0f, 66.0f, 720.0f }, { 0.f,0.f,-20.f });
    objectManager.objects.find(id)->second->body->SetVelocity(0.0f, 0.0f, -120.0f);
    objectManager.objects.find(id)->second->changeTexture(1);

    id = objectManager.AddObject(SPHERE, renderer, { 0.0f, 1.5f, 28.0f }, { 0.0f, 0.0f, -20.0f }, LightMass, { 1.2f,1.2f,1.2f });
    objectManager.objects.find(id)->second->body->SetVelocity(0.0f, 0.0f, -20.0f);

    id = objectManager.AddObject(BOX, renderer, { -50.0f, 10.0f, 0.0f }, {35.f,0.f,0.f});
    id = objectManager.AddObject(BOX, renderer, { 50.0f, 15.0f, 0.0f }, {-25.f,0.f,0.f});
    id = objectManager.AddObject(BOX, renderer, { 0.0f, 50.0f, -100.0f }, {0.f,0.f,10.f}, HeavyMass, {2.5f,2.5f,2.f});
    id = objectManager.AddObject(BOX, renderer, { 5.0f, 5.0f, -180.0f }, {0.f,0.f,80.f});
    id = objectManager.AddObject(BOX, renderer, { 0.0f, 5.0f, -180.0f }, {0.f,0.f,80.f});
    id = objectManager.AddObject(BOX, renderer, { -5.0f, 5.0f, -180.0f }, { 0.0f, 0.0f, 80.0f });
    id = objectManager.AddObject(SPHERE, renderer, { -180.f, 5.0f, 5.0f }, { 80.0f, 0.0f, 0.f });
    id = objectManager.AddObject(SPHERE, renderer, { -180.f,5.0f, -5.0f, }, { 80.0f, 0.0f, 0.f }, LightMass, {1.2f,1.2f,1.2f});
    id = objectManager.AddObject(BOX, renderer, { -180.0f, 10.0f, -180.0f }, { 78.0f, 0.0f, 78.0f });
    id = objectManager.AddObject(BOX, renderer, { 180.0f, 10.0f, 180.0f }, { 7.f, 0.0f, 7.f }, LightMass, {2.5f,2.5f,2.5f});
    id = objectManager.AddObject(BOX, renderer, { 0.0f, 5.0f, -150.0f }, {2.f,0.f,17.f}, LightMass, {1.5f,2.5f,1.5f});
    objectManager.objects.find(id)->second->changeTexture(1);

    id = objectManager.AddObject(BOX, renderer, { 0.0f, 15.0f, -190.0f }, {0.f,0.f,50.f}, LightMass, { .5f,.5f,.5f });
    objectManager.objects.find(id)->second->changeTexture(1);

    id = objectManager.AddObject(SPHERE, renderer, { 10.0f, 50.0f, 1000.0f }, { 0.f,0.f,-280.f }, HeavyMass,{2.5f,2.5f,2.5f});
    objectManager.objects.find(id)->second->changeTexture(4);
    id = objectManager.AddObject(SPHERE, renderer, { 10.0f, 50.0f, -1000.0f }, { 0.f,0.f,280.f }, HeavyMass, { 1.7f,1.7f,1.7f });

    id = objectManager.AddObject(SPHERE, renderer, { 10.0f, 60.0f, 1000.0f }, { 0.f,0.f,-280.f }, HeavyMass);
    objectManager.objects.find(id)->second->changeTexture(3);
    id = objectManager.AddObject(SPHERE, renderer, { 10.0f, 60.0f, -1000.0f }, { 0.f,0.f,280.f }, HeavyMass);

}