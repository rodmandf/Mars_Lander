#pragma once
#include "Config.h"
#include <vector>

class PhysicsEngine {
public:
    void init(float startX, float startY, float mainFuel, const std::vector<float>& aux);
    void update(ControlOutput input, float terrainHeight);
    RoverState getState() const;

    void setWind(sf::Vector2f w);
    sf::Vector2f getWind() const { return windForce; }

private:
    RoverState state;
    sf::Vector2f windForce{0.0f, 0.0f};
};
