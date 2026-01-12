#include "Config.h"
#include "TerrainGenerator.h"
#include "PhysicsEngine.h"
#include "LandingController.h"
#include "RadarTypes.h"
#include "LandingSiteDetector.h"
#include "Visualizer.h"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <ctime>
#include <algorithm>

int main() {
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    sf::RenderWindow window(sf::VideoMode({Config::WINDOW_WIDTH, Config::WINDOW_HEIGHT}), "Mars Lander - Wind Control Added");
    window.setFramerateLimit(60);

    sf::View defaultView = window.getDefaultView();
    sf::View zoomView = defaultView;
    bool zoomed = false;
    zoomView.zoom(0.5f);

    TerrainGenerator terrainGen;
    PhysicsEngine physics;
    LandingController autopilot;
    Visualizer visualizer;

    std::vector<float> terrain;
    bool autoMode = true;
    bool paused = false;

    float leftGimbal = 0.0f;
    float rightGimbal = 0.0f;
    bool landingFoundShown = false;
    float foundMsgTimer = 0.0f;

    float currentWind = 0.0f;

    RadarConfig radarCfg;
    DetectorConfig detCfg;
    detCfg.maxSlope = std::tan(Config::MAX_LANDING_ANGLE_RAD);
    detCfg.maxBandY = std::max(detCfg.maxBandY, detCfg.maxSlope * detCfg.minLenX);

    auto restartMission = [&]() {
        landingFoundShown = false;
        foundMsgTimer = 0.0f;
        autopilot.reset();

        currentWind = 0.0f; 
        physics.setWind(currentWind);

        int seed = std::rand();
        std::cout << "--- New Mission ---\n";
        
        terrain = terrainGen.generate(Config::WINDOW_WIDTH, seed);
        float startX = 100.0f + (std::rand() % (Config::WINDOW_WIDTH - 200));
        
        physics.init(startX, 50.0f, 500.0f, {100.0f, 100.0f});
    };

    restartMission();
    
    while (window.isOpen()) {
        while (const std::optional event = window.pollEvent()) {
            if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
                if (keyPressed->code == sf::Keyboard::Key::P) paused = !paused;
                if (keyPressed->code == sf::Keyboard::Key::R) restartMission();
                if (keyPressed->code == sf::Keyboard::Key::A) autoMode = !autoMode;

                // Управление ветром
                if (keyPressed->code == sf::Keyboard::Key::Q) {
                    currentWind -= 5.0f;
                    std::cout << "Wind changed: " << currentWind << "\n";
                }
                if (keyPressed->code == sf::Keyboard::Key::E) {
                    currentWind += 5.0f;
                    std::cout << "Wind changed: " << currentWind << "\n";
                }
            }
            if (const auto mb = event->getIf<sf::Event::MouseButtonPressed>()) {
                if (mb->button == sf::Mouse::Button::Left) {
                    if (!zoomed) {
                        sf::Vector2i pixelPos(mb->position.x, mb->position.y);
                        sf::Vector2f worldPos = window.mapPixelToCoords(pixelPos, defaultView);
                        zoomView.setCenter(worldPos);
                        window.setView(zoomView);
                        zoomed = true;
                    } else {
                        window.setView(defaultView);
                        zoomed = false;
                    }
                }
            }
            if (event->is<sf::Event::Closed>()) window.close();
        }

        RoverState state = physics.getState();
        int tIdx = std::clamp((int)state.x, 0, Config::WINDOW_WIDTH - 1);
        float terrainH = terrain[tIdx];

        ControlOutput ctrl{};
        Vec2 radarOrigin{state.x, state.y};
        auto radarHits = scanRadar(terrain, radarOrigin, 0.0f, radarCfg);
        auto sites = detectLandingSites(radarHits, state.x, detCfg);
        LandingSite bestSite{};
        bool hasBestSite = pickBestSite(sites, bestSite);

        // сохраняем площадку как место приземления
        if (hasBestSite && !autopilot.hasLandingTarget()) {
            autopilot.setLandingTarget(bestSite);
        }

        bool hasTargetSite = autopilot.hasLandingTarget();
        LandingSite targetSite{};
        if (hasTargetSite) targetSite = autopilot.getLandingTarget();

        if (!paused) {
            // Обновляем ветер
            physics.setWind(currentWind);

            if (foundMsgTimer > 0.0f) foundMsgTimer -= Config::DT;
            if (!landingFoundShown && autoMode && hasTargetSite) {
                landingFoundShown = true;
                foundMsgTimer = 5.0f;
            }
        }

        if (autoMode) {
            ctrl = autopilot.compute(state, radarHits);
        } else {
            ctrl.mainThrust = 0.0f;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up))    ctrl.mainThrust = 1.0f;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right)) ctrl.rightThrust = 1.0f;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left))  ctrl.leftThrust = 1.0f;

            const float gimbalSpeed = 1.5f;
            const float maxGimbal   = 0.8f;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::U)) rightGimbal += gimbalSpeed * Config::DT;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::J)) rightGimbal -= gimbalSpeed * Config::DT;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::T)) leftGimbal += gimbalSpeed * Config::DT;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::G)) leftGimbal -= gimbalSpeed * Config::DT;
            
            leftGimbal  = std::clamp(leftGimbal,  -maxGimbal, maxGimbal);
            rightGimbal = std::clamp(rightGimbal, -maxGimbal, maxGimbal);
            ctrl.leftGimbal  = leftGimbal;
            ctrl.rightGimbal = rightGimbal;
        }

        if (!paused) {
            physics.update(ctrl, terrainH);
        }

        window.clear();
        visualizer.draw(window, state, terrain, radarHits, hasTargetSite, targetSite,
                        autoMode, paused, foundMsgTimer, currentWind);
        

        window.display();
    }
    return 0;
}
