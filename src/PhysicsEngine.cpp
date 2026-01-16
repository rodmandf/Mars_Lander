#include "PhysicsEngine.h"
#include <algorithm>
#include <cmath>

void PhysicsEngine::init(float startX, float startY, float mainFuel, const std::vector<float>& aux) {
    state.x = startX; state.y = startY;
    state.vx = 0.0f; state.vy = 0.0f;
    state.angle = 0.0f; state.angularVel = 0.0f;
    state.fuelMain = mainFuel;
    state.auxTanks = aux;
    state.crashed = false;
    state.landed  = false;
    state.mainThrust = 0.0f;
    state.leftThrust = 0.0f;
    state.rightThrust = 0.0f;
    state.leftGimbal = 0.0f;
    state.rightGimbal = 0.0f;
}

void PhysicsEngine::setWind(sf::Vector2f w) {
    windForce = w;
}

RoverState PhysicsEngine::getState() const { return state; }

void PhysicsEngine::update(ControlOutput input, float terrainHeight) {
    if (state.crashed || state.landed) return;

    // Перекачка топлива
    if (state.fuelMain < 100.0f) {
        for (float& tank : state.auxTanks) {
            if (tank > 0) {
                float transfer = std::min(tank, 5.0f); 
                tank -= transfer;
                state.fuelMain += transfer;
                break; 
            }
        }
    }

    const float dt = Config::DT;
    const float mass = 10.0f;
    const float groundoffset = 20.0f; 
    const float w = 20.0f;
    const float h = 16.0f;
    const float I = (1.0f / 12.0f) * mass * (w*w + h*h); 

    float mainN  = std::clamp(input.mainThrust, 0.0f, 1.0f) * Config::MAX_MAIN_THRUST;
    float leftN  = std::clamp(input.leftThrust, 0.0f, 1.0f) * Config::MAX_SIDE_THRUST;
    float rightN = std::clamp(input.rightThrust,0.0f, 1.0f) * Config::MAX_SIDE_THRUST;

    float consumption = (mainN * 0.05f + (leftN + rightN) * 0.05f) * dt;
    state.fuelMain = std::max(0.0f, state.fuelMain - consumption);
    if (state.fuelMain <= 0.0f) {
        mainN = 0.0f; leftN = 0.0f; rightN = 0.0f;
    }

    float leftA = input.leftGimbal; 
    float rightA = input.rightGimbal;

    float Fm_xL = 0.0f;
    float Fm_yL = mainN;
    float Fl_xL = leftN * std::cos(leftA);
    float Fl_yL = leftN * std::sin(leftA);
    float Fr_xL = -rightN * std::cos(rightA);
    float Fr_yL = rightN * std::sin(rightA);

    float F_xL = Fm_xL + Fl_xL + Fr_xL;
    float F_yL = Fm_yL + Fl_yL + Fr_yL;

    float c = std::cos(state.angle);
    float s = std::sin(state.angle);

    float F_x = F_xL * c + F_yL * s;
    float F_y = -F_xL * s + F_yL * c;

    float ax = (F_x / mass);
    float ay = (F_y / mass) - Config::GRAVITY;

    ax += (windForce.x / mass);
    ay += (-windForce.y / mass);

    const float linearDragX = 0.35f;
    const float linearDragY = 0.10f;
    ax += -linearDragX * state.vx;
    ay += -linearDragY * state.vy;

    state.vx += ax * dt;
    state.vy += ay * dt;
    state.x += state.vx * dt;
    state.y -= state.vy * dt;

    float rL_xL = -w * 0.5f, rL_yL = 0.0f;
    float rR_xL = +w * 0.5f, rR_yL = 0.0f;

    float rL_x = rL_xL * c + rL_yL * s;
    float rL_y = -rL_xL * s + rL_yL * c;
    float rR_x = rR_xL * c + rR_yL * s;
    float rR_y = -rR_xL * s + rR_yL * c;

    float Fl_x = Fl_xL * c + Fl_yL * s;
    float Fl_y = -Fl_xL * s + Fl_yL * c;
    float Fr_x = Fr_xL * c + Fr_yL * s;
    float Fr_y = -Fr_xL * s + Fr_yL * c;

    float tau = (rL_x * Fl_y - rL_y * Fl_x) + (rR_x * Fr_y - rR_y * Fr_x);
    float alpha = tau / I; 

    state.angularVel += alpha * dt;
    state.angularVel *= 0.90f;
    state.angle += state.angularVel * dt;

    // Проверка посадки
    if (state.y >= terrainHeight - groundoffset) { 
        state.y = terrainHeight - groundoffset;
        
        bool safeSpeed = std::abs(state.vy) < 12.0f && std::abs(state.vx) < 12.0f; 
        
        bool bothLegsDown = std::abs(state.angle) < Config::MAX_LANDING_ANGLE_RAD; 

        if (safeSpeed && bothLegsDown) state.landed = true;
        else state.crashed = true; 

        state.vx = 0; state.vy = 0; state.angularVel = 0;
    }

    state.mainThrust = input.mainThrust;
    state.leftThrust = input.leftThrust;
    state.rightThrust = input.rightThrust;
    state.leftGimbal = input.leftGimbal;
    state.rightGimbal = input.rightGimbal;
}
