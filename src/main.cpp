#include "Config.h"
#include "TerrainGenerator.h"
#include "PhysicsEngine.h"
#include "LandingController.h"
#include "RadarTypes.h"
#include "LandingSiteDetector.h"
#include "Visualizer.h"
#include <SFML/Graphics.hpp>
#include <vector>
#include <ctime>
#include <algorithm>
#include <cmath>

int main() {
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    sf::RenderWindow window(sf::VideoMode({Config::WINDOW_WIDTH, Config::WINDOW_HEIGHT}), "Mars Lander", sf::Style::Titlebar | sf::Style::Close);
    window.setFramerateLimit(0);

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
    bool paused = true;

    float timeScale = 1.0f;
    float timeAcc = 0.0f;
    int gimbalMode = 3;       // 1=левый,2=правый,3=оба

    float leftGimbal = 0.0f;
    float rightGimbal = 0.0f;
    bool landingFoundShown = false;
    float foundMsgTimer = 0.0f;

    sf::Vector2f wind{0.0f, 0.0f};

    RadarConfig radarCfg;
    DetectorConfig detCfg;
    detCfg.maxSlope = std::tan(Config::MAX_LANDING_ANGLE_RAD);
    detCfg.maxBandY = std::max(detCfg.maxBandY, detCfg.maxSlope * detCfg.minLenX);

    auto restartMission = [&]() {
        landingFoundShown = false;
        foundMsgTimer = 0.0f;
        autopilot.reset();
        timeAcc = 0.0f;

        wind = {0.0f, 0.0f};
        physics.setWind(wind);

        int seed = std::rand();
        
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
                if (keyPressed->code == sf::Keyboard::Key::M) autoMode = !autoMode;
                if (keyPressed->code == sf::Keyboard::Key::Num0 ||
                    keyPressed->code == sf::Keyboard::Key::Numpad0)
                {
                    wind = {0.0f, 0.0f};
                }

                if (keyPressed->code == sf::Keyboard::Key::Hyphen) {
                    timeScale = std::max(0.25f, timeScale * 0.5f);
                }
                if (keyPressed->code == sf::Keyboard::Key::Equal) {
                    timeScale = std::min(4.0f, timeScale * 2.0f);
                }

                if (keyPressed->code == sf::Keyboard::Key::X) {
                    std::swap(leftGimbal, rightGimbal);
                }

                if (keyPressed->code == sf::Keyboard::Key::Z) {
                    leftGimbal = 0.0f;
                    rightGimbal = 0.0f;
                }
                if (keyPressed->code == sf::Keyboard::Key::Num1) gimbalMode = 1;
                if (keyPressed->code == sf::Keyboard::Key::Num2) gimbalMode = 2;
                if (keyPressed->code == sf::Keyboard::Key::Num3) gimbalMode = 3;
            }
            if (const auto mb = event->getIf<sf::Event::MouseButtonPressed>()) {
                if (mb->button == sf::Mouse::Button::Left) {
                    if (!zoomed) {
                        sf::Vector2i pixelPos(mb->position.x, mb->position.y);
                        sf::Vector2f worldPos = window.mapPixelToCoords(pixelPos, defaultView);
                        float viewHalfW = Config::WINDOW_WIDTH * 0.25f;
                        float viewHalfH = Config::WINDOW_HEIGHT * 0.25f;
                        float clampedX = std::clamp(worldPos.x, viewHalfW, (float)Config::WINDOW_WIDTH - viewHalfW);
                        float clampedY = std::clamp(worldPos.y, viewHalfH, (float)Config::WINDOW_HEIGHT - viewHalfH);
                        zoomView.setCenter({clampedX, clampedY});
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

        {
            const float windRate = Config::WIND_RATE;
            const float windMax = Config::WIND_MAX;
            sf::Vector2f dv{0.0f, 0.0f};
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::U)) dv.y -= windRate * Config::DT;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::J)) dv.y += windRate * Config::DT;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::H)) dv.x -= windRate * Config::DT;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::K)) dv.x += windRate * Config::DT;

            wind += dv;
            float L = std::sqrt(wind.x * wind.x + wind.y * wind.y);
            if (L > windMax && L > 1e-6f) {
                wind.x = wind.x / L * windMax;
                wind.y = wind.y / L * windMax;
            }
        }

        RoverState state = physics.getState();
        std::vector<RayHit> radarHits;
        bool hasTargetSite = autopilot.hasLandingTarget();
        LandingSite targetSite{};
        if (hasTargetSite) targetSite = autopilot.getLandingTarget();

        auto doOneSimStep = [&]() {

            physics.setWind(wind);


            RoverState st = physics.getState();
            int tIdx = std::clamp((int)st.x, 0, Config::WINDOW_WIDTH - 1);
            float terrainH = terrain[tIdx];


            Vec2 radarOrigin{st.x, st.y};
            radarHits = scanRadar(terrain, radarOrigin, st.angle, radarCfg);

            auto sites = detectLandingSites(radarHits, st.x, detCfg);
            LandingSite bestSite{};
            bool hasBestSite = pickBestSite(sites, bestSite);

            if (hasBestSite && !autopilot.hasLandingTarget()) {
                autopilot.setLandingTarget(bestSite);
            }

            hasTargetSite = autopilot.hasLandingTarget();
            if (hasTargetSite) targetSite = autopilot.getLandingTarget();


            if (foundMsgTimer > 0.0f) foundMsgTimer -= Config::DT;
            if (!landingFoundShown && autoMode && hasTargetSite) {
                landingFoundShown = true;
                foundMsgTimer = 5.0f;
            }

            ControlOutput ctrl{};
            if (autoMode) {
                ctrl = autopilot.compute(st, radarHits);
            } else {
                ctrl.mainThrust = 0.0f;
                ctrl.leftThrust = 0.0f;
                ctrl.rightThrust = 0.0f;
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up))    ctrl.mainThrust = 1.0f;
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right)) ctrl.rightThrust = 1.0f;
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left))  ctrl.leftThrust = 1.0f;

                const float gimbalSpeed = 2.5f;
                const float maxGimbal   = 0.8f;

                if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) {
                    leftGimbal  += gimbalSpeed * Config::DT;
                    rightGimbal += gimbalSpeed * Config::DT;
                }
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) {
                    leftGimbal  -= gimbalSpeed * Config::DT;
                    rightGimbal -= gimbalSpeed * Config::DT;
                }

                if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Q)) {
                    if (gimbalMode == 1) leftGimbal -= gimbalSpeed * Config::DT;
                    else if (gimbalMode == 2) rightGimbal -= gimbalSpeed * Config::DT;
                    else { leftGimbal -= gimbalSpeed * Config::DT; rightGimbal += gimbalSpeed * Config::DT; }
                }
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::E)) {
                    if (gimbalMode == 1) leftGimbal += gimbalSpeed * Config::DT;
                    else if (gimbalMode == 2) rightGimbal += gimbalSpeed * Config::DT;
                    else { leftGimbal += gimbalSpeed * Config::DT; rightGimbal -= gimbalSpeed * Config::DT; }
                }

                leftGimbal  = std::clamp(leftGimbal,  -maxGimbal, maxGimbal);
                rightGimbal = std::clamp(rightGimbal, -maxGimbal, maxGimbal);
                ctrl.leftGimbal  = leftGimbal;
                ctrl.rightGimbal = rightGimbal;
            }

            physics.update(ctrl, terrainH);

            state = physics.getState();
        };

        if (!paused) {
            timeAcc += timeScale;
            while (timeAcc >= 1.0f) {
                doOneSimStep();
                timeAcc -= 1.0f;
            }
        } else {
            Vec2 radarOrigin{state.x, state.y};
            radarHits = scanRadar(terrain, radarOrigin, 0.0f, radarCfg);
            hasTargetSite = autopilot.hasLandingTarget();
            if (hasTargetSite) targetSite = autopilot.getLandingTarget();
        }

        window.clear();
        visualizer.draw(window, state, terrain, radarHits, hasTargetSite, targetSite,
                        autoMode, paused, foundMsgTimer, wind,
                        timeScale, gimbalMode, autopilot.getPhaseName());
        

        window.display();
    }
    return 0;
}
