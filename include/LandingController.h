#pragma once
#include "Config.h"

class LandingController {
public:
    ControlOutput compute(const RoverState& state, const std::vector<float>& terrain);

private:
    enum class Phase { Approach, Hover, Descend };
    Phase phase = Phase::Approach;

    float stableHoverTimer = 0.0f; // seconds

    float integralAlt = 0.0f;
    float prevErrorAlt = 0.0f;

    void resetVerticalPid() { integralAlt = 0.0f; prevErrorAlt = 0.0f; }
};
