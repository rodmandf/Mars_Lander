#pragma once
#include <SFML/Graphics.hpp>
#include "Config.h"
#include "RadarTypes.h"
#include "LandingSiteDetector.h"
#include <vector>

class Visualizer {
public:
    Visualizer();

    void draw(sf::RenderWindow& window, const RoverState& state, 
              const std::vector<float>& terrain, 
              const std::vector<RayHit>& radarHits,
              bool hasTargetSite,
              const LandingSite& targetSite,
              bool autoMode, bool paused,
              float foundMsgTimer,
              float currentWind,
              float timeScale,
              int gimbalMode,
              const char* phaseName); 

private:
    sf::Font font;
    std::vector<sf::Vector2f> stars;

    void drawHUD(sf::RenderWindow& window,
                 const RoverState& state,
                 bool autoMode,
                 bool paused,
                 float currentWind,
                 float timeScale,
                 int gimbalMode,
                 const char* phaseName,
                 bool hasTargetSite,
                 const LandingSite& targetSite);
};
