#include "Visualizer.h"
#include <iostream>
#include <cstdlib>
#include <algorithm>

Visualizer::Visualizer() {
    std::vector<std::string> fontPaths;

    #ifdef _WIN32
        fontPaths.push_back("C:/Windows/Fonts/arial.ttf");
    #elif defined(__APPLE__)
        fontPaths.push_back("/Library/Fonts/Arial.ttf");
        fontPaths.push_back("/System/Library/Fonts/Helvetica.ttc");
        fontPaths.push_back("/System/Library/Fonts/Geneva.ttf"); 
    #elif defined(__linux__)
        fontPaths.push_back("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");
        fontPaths.push_back("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf");
        fontPaths.push_back("/usr/share/fonts/truetype/freefont/FreeSans.ttf");
        fontPaths.push_back("/usr/share/fonts/truetype/msttcorefonts/Arial.ttf"); 
    #endif


    bool fontLoaded = false;
    for (const auto& path : fontPaths) {
        if (font.openFromFile(path)) {
            fontLoaded = true;
            std::cout << "Loaded font: " << path << std::endl;
            break;
        }
    }

    if (!fontLoaded) {
        std::cerr << "Failed to load any system font!" << std::endl;
    }
    for (int i = 0; i < 100; i++) {
        stars.push_back({(float)(std::rand() % Config::WINDOW_WIDTH), (float)(std::rand() % Config::WINDOW_HEIGHT)});
    }
}

void Visualizer::draw(sf::RenderWindow& window, const RoverState& state, 
                      const std::vector<float>& terrain, 
                      const std::vector<int>& zones, 
                      bool autoMode, bool paused, int scanX, int highlightZoneX,
                      float foundMsgTimer, bool scanActive)
{
    // 1. Небо
    sf::VertexArray sky(sf::PrimitiveType::TriangleStrip, 4);
    sky[0] = sf::Vertex{ sf::Vector2f(0.f, 0.f), Config::MARS_SKY_TOP };
    sky[1] = sf::Vertex{ sf::Vector2f((float)Config::WINDOW_WIDTH, 0.f), Config::MARS_SKY_TOP };
    sky[2] = sf::Vertex{ sf::Vector2f(0.f, (float)Config::WINDOW_HEIGHT), Config::MARS_SKY_BOTTOM };
    sky[3] = sf::Vertex{ sf::Vector2f((float)Config::WINDOW_WIDTH, (float)Config::WINDOW_HEIGHT), Config::MARS_SKY_BOTTOM };
    window.draw(sky);

    sf::VertexArray starPoints(sf::PrimitiveType::Points, stars.size());
    for (size_t i = 0; i < stars.size(); i++) {
        starPoints[i].position = stars[i];
        starPoints[i].color = sf::Color(255, 255, 255, 150);
    }
    window.draw(starPoints);

    // 3. Ландшафт
    if (!terrain.empty()) {
        sf::VertexArray ground(sf::PrimitiveType::TriangleStrip, terrain.size() * 2);
        for (size_t i = 0; i < terrain.size(); i++) {
            float h = terrain[i];
            ground[i * 2].position = sf::Vector2f((float)i, h);
            ground[i * 2].color = Config::TERRAIN_COLOR_TOP;
            ground[i * 2 + 1].position = sf::Vector2f((float)i, (float)Config::WINDOW_HEIGHT);
            ground[i * 2 + 1].color = Config::TERRAIN_COLOR_BOTTOM;
        }
        window.draw(ground);
    }


    if (scanActive && autoMode && scanX >= 0 && scanX < (int)terrain.size()) {
        sf::VertexArray scan(sf::PrimitiveType::Lines, 2);
        scan[0].position = {(float)scanX, 0.f};
        scan[1].position = {(float)scanX, terrain[scanX]};
        scan[0].color = scan[1].color = sf::Color(0, 255, 255, 140);
        window.draw(scan);
    }


    // 4. Зоны посадки
    for (int zoneX : zones) {
        if (zoneX >= 0 && zoneX < (int)terrain.size()) {
            sf::RectangleShape zoneMark({60.f, 5.f}); 
            zoneMark.setOrigin({30.f, 0.f});
            zoneMark.setPosition({(float)zoneX, terrain[zoneX]});
            zoneMark.setFillColor(sf::Color(255, 255, 0, 100));
            window.draw(zoneMark);
        }
    }

    
    sf::Transform t;
    t.translate({state.x, state.y});
    t.rotate(sf::degrees(state.angle * 180.0f / 3.14159f));

    sf::RectangleShape body({20.f, 16.f});
    body.setOrigin({10.f, 8.f});
    body.setFillColor(sf::Color(200, 200, 220));
    body.setOutlineColor(sf::Color::Black);
    body.setOutlineThickness(1.f);
    window.draw(body, t);

    // Ноги
    sf::RectangleShape legL({4.f, 12.f});
    legL.setOrigin({2.f, 0.f});
    legL.setPosition({-8.f, 8.f});
    legL.setRotation(sf::degrees(30.f));
    legL.setFillColor(sf::Color(50, 50, 50));
    window.draw(legL, t);

    sf::RectangleShape legR({4.f, 12.f});
    legR.setOrigin({2.f, 0.f});
    legR.setPosition({8.f, 8.f});
    legR.setRotation(sf::degrees(-30.f));
    legR.setFillColor(sf::Color(50, 50, 50));
    window.draw(legR, t);

    // Главный двигатель
    if (state.fuelMain > 0 && state.mainThrust > 0.01f && !state.crashed && !state.landed) {
        // Размер пламени зависит от тяги
        float thrustScale = state.mainThrust; 
        float flicker = 10.0f + (std::rand() % 15);
        
        sf::ConvexShape flame;
        flame.setPointCount(3);
        flame.setPoint(0, { -5.f, 8.f });
        flame.setPoint(1, { 5.f, 8.f });
        flame.setPoint(2, { 0.f, 8.f + flicker * thrustScale });
        flame.setFillColor(sf::Color(255, 100, 0));
        window.draw(flame, t);
    }

    // БОКОВЫЕ ДВИГАТЕЛИ
    auto drawSideJet = [&](sf::Vector2f mount, float throttle, float gimbal, bool isLeft)
    {
        if (throttle <= 0.01f) return;

        sf::RectangleShape noz({6.f, 3.f});
        noz.setOrigin({3.f, 1.5f});
        noz.setPosition(mount);
        noz.setFillColor(sf::Color(220,220,220,220));
        window.draw(noz, t);

        sf::Vector2f dirLocal;
        if (isLeft)  dirLocal = { std::cos(gimbal),  std::sin(gimbal) };
        else dirLocal = { -std::cos(gimbal), std::sin(gimbal) };

        sf::Vector2f exhaustDir = -dirLocal;

        float L = 12.f + 20.f * throttle;

        sf::VertexArray jet(sf::PrimitiveType::Lines, 2);
        jet[0].position = mount;
        jet[1].position = mount + exhaustDir * L;
        jet[0].color = sf::Color(255,255,255,200);
        jet[1].color = sf::Color(255,180,100,60);
        window.draw(jet, t);
    };

    drawSideJet({-12.f, 0.f}, state.leftThrust,  state.leftGimbal,  true);
    drawSideJet({+12.f, 0.f}, state.rightThrust, state.rightGimbal, false);

    drawHUD(window, state, autoMode, paused);

    if (foundMsgTimer > 0.0f && highlightZoneX >= 0 && highlightZoneX < (int)terrain.size()) {
        sf::Text msg(font);
        msg.setCharacterSize(16);
        msg.setFillColor(sf::Color(0, 255, 0, 230));

        msg.setString("landing surface found coordinates are: x=" +
                    std::to_string(highlightZoneX) +
                    " y=" + std::to_string((int)terrain[highlightZoneX]));

        msg.setPosition({20.f, 200.f}); 
        window.draw(msg);
    }

}

void Visualizer::drawHUD(sf::RenderWindow& window, const RoverState& state, bool autoMode, bool paused) {

    sf::RectangleShape panel({250.f, 180.f});
    panel.setPosition({10.f, 10.f});
    panel.setFillColor(sf::Color(0, 0, 0, 150));
    panel.setOutlineColor(sf::Color::White);
    panel.setOutlineThickness(1.f);
    window.draw(panel);

    sf::Text txt(font);
    txt.setCharacterSize(14);
    txt.setFillColor(sf::Color::White);
    txt.setPosition({20.f, 20.f});

    std::string status = "FLYING";
    if (state.crashed) status = "CRASHED";
    else if (state.landed) status = "LANDED SUCCESS";

    std::string info = 
        "Status: " + status + "\n" +
        "Mode: " + (autoMode ? "AUTOPILOT" : "MANUAL") + "\n" +
        "Alt: " + std::to_string((int)(Config::WINDOW_HEIGHT - 150 - state.y)) + "\n" +
        "Speed X: " + std::to_string((int)state.vx) + "\n" +
        "Speed Y: " + std::to_string((int)state.vy) + "\n" +
        "Fuel: " + std::to_string((int)state.fuelMain);
    
    if (paused) {
        sf::Text pausedTxt(font);
        pausedTxt.setString("PAUSED");
        pausedTxt.setCharacterSize(48);
        pausedTxt.setFillColor(sf::Color(255, 255, 255, 230));

        auto bounds = pausedTxt.getLocalBounds();
        pausedTxt.setOrigin({bounds.position.x + bounds.size.x * 0.5f,
                            bounds.position.y + bounds.size.y * 0.5f});
        pausedTxt.setPosition({Config::WINDOW_WIDTH * 0.5f,
                            Config::WINDOW_HEIGHT * 0.5f});

        window.draw(pausedTxt);
    }

    txt.setString(info);
    window.draw(txt);

    for (size_t i = 0; i < state.auxTanks.size(); i++) {
        float w = state.auxTanks[i];
        sf::RectangleShape bar({w / 2.0f, 8.f});
        bar.setPosition({20.f, 145.f + i * 15.f});
        bar.setFillColor(w > 0 ? sf::Color::Cyan : sf::Color::Red);
        window.draw(bar);
    }
    
}