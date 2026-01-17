#pragma once
#include "Config.h"
#include "RadarTypes.h"
#include "LandingSiteDetector.h"

class LandingController {
public:
    ControlOutput compute(const RoverState& state, const std::vector<RayHit>& radarHits);

    // Сохраняем найденную площадку
    void setLandingTarget(const LandingSite& site);
    void clearLandingTarget();
    bool hasLandingTarget() const;
    const LandingSite& getLandingTarget() const;

    void reset();

    const char* getPhaseName() const;

private:
    enum class Phase { Approach, Hover, Descend };
    Phase phase = Phase::Approach;

    float stableHoverTimer = 0.0f;
    float hoverDuration = 0.0f;
    
    // PID высоты
    float integralAlt = 0.0f;
    float prevErrorAlt = 0.0f;

    // PID горизонтальной скорости
    float integralVx = 0.0f; 

    bool targetLocked = false;
    LandingSite lockedSite{};

    void resetPids() { 
        integralAlt = 0.0f; 
        prevErrorAlt = 0.0f;
        integralVx = 0.0f; 
    }
};
