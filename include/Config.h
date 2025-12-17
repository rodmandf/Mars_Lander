#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <optional>

// Константы физики и мира 
namespace Config {
    const float GRAVITY = 3.711f;
    const float DT = 1.0f / 25.0f;
    const int WINDOW_WIDTH = 1280;
    const int WINDOW_HEIGHT = 720;
    const float MAX_MAIN_THRUST = 100.0f; // Мощность двигателя
    const float MAX_SIDE_THRUST = 20.0f;
    
    // Цвета
    const sf::Color MARS_SKY_TOP(20, 20, 40);
    const sf::Color MARS_SKY_BOTTOM(80, 40, 30);
    const sf::Color TERRAIN_COLOR_TOP(200, 100, 50);
    const sf::Color TERRAIN_COLOR_BOTTOM(50, 20, 10);
}

// Общие структуры данных
struct RoverState {
    float x, y;
    float vx, vy;
    float angle;  // Радианы
    float angularVel;
    float fuelMain;    // Основной бак
    std::vector<float> auxTanks; // Доп. баки
    bool crashed = false;
    bool landed = false;
    float mainThrust = 0.0f; 
    float sideThrust = 0.0f;
    float leftThrust = 0.0f;
    float rightThrust = 0.0f;
    float leftGimbal = 0.0f;
    float rightGimbal = 0.0f;

};

struct ControlOutput {
    float mainThrust;      

    float leftThrust;      
    float rightThrust;     

    float leftGimbal;     
    float rightGimbal;
};

