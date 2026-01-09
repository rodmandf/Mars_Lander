#include "LandingController.h"
#include <algorithm>
#include <cmath>
#include <vector>
#include <iostream>

// Функция поиска зоны
int scanForNearestZone(const std::vector<float>& terrain, float roverX) {
    int bestX = -1;
    float bestDist = 1e9f;
    const float flatEps = 0.1f;
    const int minFlatLen = 40;
    int flatStart = 0;
    int flatLen = 0;

    for (int x = 1; x < (int)terrain.size(); ++x) {
        if (std::abs(terrain[x] - terrain[x - 1]) < flatEps) {
            if (flatLen == 0) flatStart = x - 1;
            flatLen++;
        } else {
            if (flatLen > minFlatLen) {
                int center = flatStart + flatLen / 2;
                float d = std::abs((float)center - roverX);
                if (d < bestDist) { bestDist = d; bestX = center; }
            }
            flatLen = 0;
        }
    }
    if (flatLen > minFlatLen) {
        int center = flatStart + flatLen / 2;
        float d = std::abs((float)center - roverX);
        if (d < bestDist) { bestDist = d; bestX = center; }
    }
    return bestX;
}

ControlOutput LandingController::compute(const RoverState& state, const std::vector<float>& terrain) {
    ControlOutput out{};
    out.leftGimbal = 0.0f;
    out.rightGimbal = 0.0f;

    const float groundOffset = 12.0f;
    int targetX = scanForNearestZone(terrain, state.x);
    if (targetX == -1) targetX = (int)state.x;

    int tx = std::clamp(targetX, 0, (int)terrain.size() - 1);
    float targetTerrainH = terrain[tx];
    float altToTarget = (targetTerrainH - groundOffset) - state.y;

    float distX = (float)targetX - state.x;

    const float hoverAlt = 120.0f; 
    const float xTol = 12.0f; 
    const float vxTol = 15.0f; 
    
    const float angVelTol = 0.3f;
    const float stableTimeToDescend = 1.0f; 

    if (phase == Phase::Descend && std::abs(distX) > 5.0f * xTol) { 
        phase = Phase::Approach;
        resetPids();
        hoverDuration = 0.0f;
    }

    if (phase == Phase::Approach) {
        hoverDuration = 0.0f; 
        if (std::abs(distX) < 2.0f * xTol && std::abs(state.vx) < vxTol) {
            phase = Phase::Hover;
            resetPids();
        }
    } 
    else if (phase == Phase::Hover) {
        hoverDuration += Config::DT;
        bool stableRotation = (std::abs(state.angularVel) < angVelTol);
        bool stablePos = (std::abs(distX) < xTol) && (std::abs(state.vx) < 8.0f); 

        if (stablePos && stableRotation) {
            stableHoverTimer += Config::DT;
        } else {
            stableHoverTimer = 0.0f;
        }

        bool perfectCondition = (stableHoverTimer >= stableTimeToDescend);
        bool timeoutCondition = (hoverDuration > 6.0f) && (std::abs(state.angularVel) < 0.5f);

        if (perfectCondition || timeoutCondition) {
            phase = Phase::Descend;
            stableHoverTimer = 0.0f;
            hoverDuration = 0.0f;
            resetPids();
            std::cout << "Descend initiated.\n";
        }
    }

    // Управление по горизонтали
    float targetVx = 0.0f;
    if (phase == Phase::Approach) {
        targetVx = std::clamp(distX * 0.45f, -25.0f, 25.0f);
    } else {
        targetVx = 0.0f; 
    }

    float errorVx = targetVx - state.vx;

    if (!state.landed && !state.crashed) {
        integralVx += errorVx * Config::DT;
        integralVx = std::clamp(integralVx, -12.0f, 12.0f);
    }

    float kp_vx = 0.12f; 
    float ki_vx = 0.03f; 

    float targetAngle = errorVx * kp_vx + integralVx * ki_vx;
    targetAngle = std::clamp(targetAngle, -0.6f, 0.6f); 

    if (phase == Phase::Descend) {
        // Начинаем выравниваться уже с 40 метров
        if (altToTarget < 40.0f) {
             // Линейно уменьшаем разрешенный наклон с 40м до 8м
             float ratio = (altToTarget - 8.0f) / 32.0f; 
             ratio = std::clamp(ratio, 0.0f, 1.0f);
             targetAngle *= ratio;
        }

        if (altToTarget < 8.0f) {
            targetAngle = 0.0f;
        }
    }


    float errorAng = targetAngle - state.angle;
    auto deg2rad = [](float d) { return d * 3.1415926f / 180.0f; };
    float angDeadband = deg2rad(0.5f);
    
    if (phase == Phase::Descend && altToTarget < 15.0f) angDeadband = deg2rad(0.1f);

    if (std::abs(errorAng) < angDeadband && std::abs(state.angularVel) < 0.1f) {
        out.leftThrust = 0.0f; out.rightThrust = 0.0f;
        out.leftGimbal = 0.0f; out.rightGimbal = 0.0f;
    } else {
        float kp = 4.5f; 
        float kd = 8.0f;
        float gMax = 0.6f;
        
        if (altToTarget < 20.0f) { 
            kp = 9.0f;  
            kd = 14.0f; 
            gMax = 1.0f; 
        }

        float turnCmd = std::clamp(errorAng * kp - state.angularVel * kd, -1.0f, 1.0f);
        float thr = std::abs(turnCmd);
        float g = turnCmd * gMax;

        out.leftThrust  = thr;
        out.rightThrust = thr;
        out.leftGimbal  = -g;
        out.rightGimbal = +g;
    }

    // Вертикальное управление
    float targetVy = 0.0f;
    if (phase == Phase::Descend) {
        targetVy = -12.0f;
        if (altToTarget < 40.0f) targetVy = -5.0f;
        if (altToTarget < 12.0f) targetVy = -2.0f;
        if (altToTarget < 4.0f)  targetVy = -1.0f;
    } else {
        float altErr = (hoverAlt - altToTarget);
        targetVy = std::clamp(altErr * 0.4f, -10.0f, 10.0f);
    }

    float errorVy = targetVy - state.vy;
    integralAlt += errorVy * Config::DT;
    integralAlt = std::clamp(integralAlt, -8.0f, 8.0f);
    
    float derivAlt = (errorVy - prevErrorAlt) / Config::DT;
    prevErrorAlt = errorVy;

    float pidOut = errorVy * 0.22f + integralAlt * 0.005f + derivAlt * 0.16f;

    float baseThrust = 0.371f; 
    float cosA = std::abs(std::cos(state.angle));
    if (cosA < 0.5f) cosA = 0.5f; 
    if (cosA > 0.01f) baseThrust /= cosA;

    out.mainThrust = std::clamp(baseThrust + pidOut, 0.0f, 1.0f);

    if (altToTarget < 70.0f && state.vy < -20.0f) out.mainThrust = 1.0f;

    return out;
}
