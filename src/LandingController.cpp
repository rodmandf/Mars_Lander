#include "LandingController.h"
#include <algorithm>
#include <cmath>
#include <vector>

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
    std::cout << "Target X: " << targetX << "\n";
    std::cout << "Rover X: " << state.x << "\n";
    if (targetX == -1) targetX = (int)state.x;

    int tx = std::clamp(targetX, 0, (int)terrain.size() - 1);
    int cx = std::clamp((int)state.x, 0, (int)terrain.size() - 1);

    float targetTerrainH = terrain[tx];
    float currentTerrainH = terrain[cx];

    float altToTarget = (targetTerrainH - groundOffset) - state.y;
    std::cout << "Altitude to target: " << altToTarget << "\n";
    float altToCurrent = currentTerrainH - state.y; 

    float distX = (float)targetX - state.x;


    const float hoverAlt = 120.0f; 
    const float xTol = 10.0f; 
    const float vxTol = 12.0f; 
    const float angTol = 0.12f; 
    const float angVelTol = 0.25f;
    const float stableTimeToDescend = 0.8f; 
    
    
    if (phase == Phase::Descend && std::abs(distX) > 2.0f * xTol) {
        phase = Phase::Approach;
        resetVerticalPid();
    }

    if (phase == Phase::Approach) {
        
        if (std::abs(distX) < 2.0f *xTol && std::abs(state.vx) < vxTol) {
            phase = Phase::Hover;
            resetVerticalPid();
        }
    } else if (phase == Phase::Hover) {
        bool stableAtt = (std::abs(state.angle) < angTol) && (std::abs(state.angularVel) < angVelTol);
        bool stablePos = (std::abs(distX) < xTol) && (std::abs(state.vx) < vxTol);
        bool atHoverAlt = std::abs(altToTarget - hoverAlt) < 12.0f;



        if (stablePos) stableHoverTimer += Config::DT;
        else stableHoverTimer = 0.0f;

        bool forceDescend = (stableHoverTimer >= stableTimeToDescend) && stableAtt;


        if ((stableAtt && stablePos && atHoverAlt) || forceDescend) {
            phase = Phase::Descend;
            stableHoverTimer = 0.0f;
            resetVerticalPid();
        }


        if (stableAtt && stablePos && atHoverAlt) {
            phase = Phase::Descend;
            resetVerticalPid();
        }
    }

  
    float targetVx = 0.0f;
    if (phase == Phase::Approach) {
        targetVx = std::clamp(distX * 0.35f, -20.0f, 20.0f);
    } else {
        targetVx = 0.0f;
    }

    float errorVx = targetVx - state.vx;
    float targetAngle = std::clamp(errorVx * 0.10f, -0.35f, 0.35f);

    if (phase != Phase::Approach) {
        targetAngle = 0.0f; 
    }

 
    float errorAng = targetAngle - state.angle;


    auto deg2rad = [](float d) { return d * 3.1415926f / 180.0f; };


    float angDeadband = deg2rad(1.0f);
    if (altToTarget < 30.0f)  angDeadband = deg2rad(3.0f);
    if (altToTarget < 10.0f)  angDeadband = deg2rad(6.0f);

    if (std::abs(errorAng) < angDeadband && std::abs(state.angularVel) < 0.2f) {
        out.leftThrust = 0.0f;
        out.rightThrust = 0.0f;
        out.leftGimbal = 0.0f;
        out.rightGimbal = 0.0f;
    } else {
        float kp = 4.0f;
        float kd = 7.0f;
        float gMax = 0.6f;

        if (altToTarget < 30.0f) { kp *= 0.6f; kd *= 0.6f; gMax *= 0.6f; }
        if (altToTarget < 10.0f) { kp *= 0.4f; kd *= 0.4f; gMax *= 0.4f; }

        float turnCmd = std::clamp(errorAng * kp - state.angularVel * kd, -1.0f, 1.0f);

        float thr = std::abs(turnCmd);
        float g = turnCmd * gMax;

        out.leftThrust  = thr;
        out.rightThrust = thr;
        out.leftGimbal  = -g;
        out.rightGimbal = +g;
    }


    float targetVy = 0.0f;

    if (phase == Phase::Descend) {
    // Быстро падаем почти до земли
    targetVy = -14.0f;                 // было -8

    // Поздний "брейк": начинаем снижать скорость ближе к поверхности
    if (altToTarget < 45.0f) targetVy = -6.0f;
    if (altToTarget < 20.0f) targetVy = -2.0f;
    if (altToTarget < 8.0f)  targetVy = -1.0f;
    if (altToTarget < 4.0f)  targetVy = -0.5f;
    } else {
        float altErr = (hoverAlt - altToTarget);
        targetVy = std::clamp(altErr * 0.35f, -6.0f, 6.0f);
    }


    float errorVy = targetVy - state.vy;

    integralAlt += errorVy * Config::DT;
    integralAlt = std::clamp(integralAlt, -8.0f, 8.0f);

    float derivAlt = (errorVy - prevErrorAlt) / Config::DT;
    prevErrorAlt = errorVy;

    float pidOut = errorVy * 0.22f + integralAlt * 0.004f + derivAlt * 0.16f;

    float baseThrust = 0.371f;

    if (std::abs(std::cos(state.angle)) > 0.2f) {
        baseThrust /= std::abs(std::cos(state.angle));
    }

    out.mainThrust = std::clamp(baseThrust + pidOut, 0.0f, 1.0f);

    if (altToTarget < 60.0f && state.vy < -16.0f) out.mainThrust = 1.0f;


    return out;
}
