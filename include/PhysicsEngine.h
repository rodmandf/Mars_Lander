#pragma once
#include "Config.h"
#include <vector>

class PhysicsEngine {
public:
    void init(float startX, float startY, float mainFuel, const std::vector<float>& aux);
    void update(ControlOutput input, float terrainHeight);
    RoverState getState() const;

    void setWind(float w);

private:
    RoverState state;
    float windForce = 0.0f; // Текущая сила ветра
};
