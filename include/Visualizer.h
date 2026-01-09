#pragma once
#include <SFML/Graphics.hpp>
#include "Config.h"
#include <vector>

class Visualizer {
public:
    Visualizer();

    void draw(sf::RenderWindow& window, const RoverState& state, 
              const std::vector<float>& terrain, 
              const std::vector<int>& zones, 
              bool autoMode, bool paused, int scanX, int highlightZoneX,
              float foundMsgTimer, bool scanActive,
              float currentWind); 

private:
    sf::Font font;
    std::vector<sf::Vector2f> stars;

    void drawHUD(sf::RenderWindow& window, const RoverState& state, bool autoMode, bool paused, float currentWind);
};
