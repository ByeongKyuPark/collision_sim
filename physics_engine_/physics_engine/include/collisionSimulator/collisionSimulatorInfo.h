#pragma once
#include <vector>

struct CollisionSimulatorInfo {
    std::vector<unsigned int> selectedObjectIDs;
    float deltaTimeMultiplier;
    bool shouldRenderContactInfo;
    unsigned int newObjectID;
};

static constexpr float Flt_Epsilon = 0.001f;
static constexpr float Idle_Threshold = 0.00035f;

extern float g_Gravity;