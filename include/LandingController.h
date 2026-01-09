#pragma once
#include "Config.h"
#include <vector>

class LandingController {
public:
    ControlOutput compute(const RoverState& state, const std::vector<float>& terrain);

private:
    enum class Phase { Approach, Hover, Descend };
    Phase phase = Phase::Approach;

    float stableHoverTimer = 0.0f; // Таймер идеальной стабильности
    float hoverDuration = 0.0f;    // Общее время висения
    
    // PID высоты
    float integralAlt = 0.0f;
    float prevErrorAlt = 0.0f;

    // PID горизонтальной скорости
    float integralVx = 0.0f; 

    void resetPids() { 
        integralAlt = 0.0f; 
        prevErrorAlt = 0.0f;
        integralVx = 0.0f; 
    }
};
