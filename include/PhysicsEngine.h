#pragma once
#include "Config.h"

class PhysicsEngine {
public:
    void init(float startX, float startY, float mainFuel, const std::vector<float>& aux);
    void update(ControlOutput input, float terrainHeight);
    RoverState getState() const;

private:
    RoverState state;
};