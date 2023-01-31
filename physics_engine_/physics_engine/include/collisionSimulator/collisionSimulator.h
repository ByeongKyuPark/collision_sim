#pragma once

#include "graphics/renderer.h"
#include "object.h"
#include <unordered_map>
#include <vector>
#include <memory>//shared_ptr

#include "physics/objectManager.h"
#include "physics/body.h"
#include "physics/detector.h"
#include "physics/resolver.h"
#include "graphics/camera.h"
#include "../collisionSimulator/geometry.h"
#include "../collisionSimulator/contact_info.h"
#include "../collisionSimulator/collisionSimulatorInfo.h"//all collisionSimulator specific things that will be passed to the game object manager

class CollisionSimulator
{
private:
    Graphics::Camera camera;
    bool shouldPause;
    Graphics::Renderer renderer;

    shared_ptr<CollisionSimulatorInfo> collisionSimulatorSpecificInfo;
    ObjectManager objectManager;

public:
    CollisionSimulator();
    
    void run();

    void HandleInput();

private:
    void LoadDefaultScenario();
};
