#include "Visualizer.h"
#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <string> 
#include <cstdio>
#include <cmath>

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

    windStreaks.reserve(160);
    for (int i = 0; i < 160; ++i) {
        windStreaks.push_back({
            (float)(std::rand() % Config::WINDOW_WIDTH),
            (float)(std::rand() % Config::WINDOW_HEIGHT)
        });
    }
}

void Visualizer::draw(sf::RenderWindow& window, const RoverState& state, 
                      const std::vector<float>& terrain, 
                      const std::vector<RayHit>& radarHits,
                      bool hasTargetSite,
                      const LandingSite& targetSite,
                      bool autoMode, bool paused,
                      float foundMsgTimer,
                      sf::Vector2f wind,
                      float timeScale,
                      int gimbalMode,
                      const char* phaseName) 
{
    // Небо
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

    // Ландшафт
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

    // Лучи радара
    for (const auto& h : radarHits) {
        sf::Vector2f o{h.origin.x, h.origin.y};
        sf::Vector2f e = h.hit ? sf::Vector2f(h.point.x, h.point.y)
                               : sf::Vector2f(h.origin.x + h.dir.x * h.t, h.origin.y + h.dir.y * h.t);

        sf::VertexArray ray(sf::PrimitiveType::Lines, 2);
        ray[0].position = o;
        ray[1].position = e;
        ray[0].color = sf::Color(0, 255, 255, 110);
        ray[1].color = sf::Color(0, 255, 255, 40);
        window.draw(ray);

        if (h.hit) {
            sf::CircleShape p(2.0f);
            p.setOrigin({2.0f, 2.0f});
            p.setPosition(e);
            p.setFillColor(sf::Color(0, 255, 0, 180));
            window.draw(p);
        }
    }

    // Лучшая площадка
    if (hasTargetSite) {
        float w = std::max(1.0f, targetSite.x1 - targetSite.x0);
        sf::RectangleShape s({w, 6.f});
        s.setPosition({targetSite.x0, targetSite.yMean - 3.f});
        s.setFillColor(sf::Color(0, 255, 0, 90));
        window.draw(s);

        sf::CircleShape c(4.0f);
        c.setOrigin({4.0f, 4.0f});
        c.setPosition({targetSite.centerX, targetSite.yMean});
        c.setFillColor(sf::Color(0, 255, 0, 200));
        window.draw(c);
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

    // Боковые двигатели
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

    sf::View currentView = window.getView();
    sf::View screenView = window.getDefaultView();
    window.setView(screenView);

    {
        float dt = paused ? 0.0f : (Config::DT * timeScale);
        float windMax = Config::WIND_MAX;
        float mag = std::sqrt(wind.x * wind.x + wind.y * wind.y);
        if (mag > 0.5f) {
            float inv = 1.0f / std::max(1e-6f, mag);
            sf::Vector2f dir{ wind.x * inv, wind.y * inv };

            float speedScale = 6.0f; // визуальная скорость частиц
            sf::Vector2f v{ wind.x * speedScale, wind.y * speedScale };
            for (auto& p : windStreaks) {
                p += v * dt;
                if (p.x < 0.0f) p.x += (float)Config::WINDOW_WIDTH;
                if (p.x >= (float)Config::WINDOW_WIDTH) p.x -= (float)Config::WINDOW_WIDTH;
                if (p.y < 0.0f) p.y += (float)Config::WINDOW_HEIGHT;
                if (p.y >= (float)Config::WINDOW_HEIGHT) p.y -= (float)Config::WINDOW_HEIGHT;
            }

            float len = std::clamp(3.0f + (mag / std::max(1e-6f, windMax)) * 14.0f, 3.0f, 17.0f);
            sf::VertexArray streaks(sf::PrimitiveType::Lines, windStreaks.size() * 2);
            for (size_t i = 0; i < windStreaks.size(); ++i) {
                sf::Vector2f a = windStreaks[i];
                sf::Vector2f b = a - dir * len;
                streaks[i * 2 + 0].position = a;
                streaks[i * 2 + 1].position = b;
                streaks[i * 2 + 0].color = sf::Color(255, 255, 255, 110);
                streaks[i * 2 + 1].color = sf::Color(255, 255, 255, 20);
            }
            window.draw(streaks);
        }
    }

    drawHUD(window, state, autoMode, paused, wind, timeScale, gimbalMode, phaseName,
            hasTargetSite, targetSite);

    if (foundMsgTimer > 0.0f && hasTargetSite) {
        sf::Text msg(font);
        msg.setCharacterSize(16);
        msg.setFillColor(sf::Color(0, 255, 0, 230));
        msg.setString("Landing site found: x=" +
                      std::to_string((int)targetSite.centerX) +
                      " y=" + std::to_string((int)targetSite.yMean));
        msg.setPosition({20.f, 220.f});
        window.draw(msg);
    }

    window.setView(currentView);
}

void Visualizer::drawHUD(sf::RenderWindow& window,
                         const RoverState& state,
                         bool autoMode,
                         bool paused,
                         sf::Vector2f wind,
                         float timeScale,
                         int gimbalMode,
                         const char* phaseName,
                         bool hasTargetSite,
                         const LandingSite& targetSite)
{
    float panelHeight = 260.f;
    if (hasTargetSite) panelHeight += 50.f;

    sf::RectangleShape panel({270.f, panelHeight}); 
    panel.setPosition({10.f, 10.f});
    panel.setFillColor(sf::Color(0, 0, 0, 150));
    panel.setOutlineColor(sf::Color::White);
    panel.setOutlineThickness(1.f);
    window.draw(panel);

    std::string status = "FLYING";
    if (state.crashed) status = "CRASHED";
    else if (state.landed) status = "LANDED SUCCESS";

    char timeScaleStr[16];
    std::snprintf(timeScaleStr, sizeof(timeScaleStr), "%.2fx", timeScale);

    float angleDeg = state.angle * 180.0f / 3.14159265f;
    char angleStr[16];
    std::snprintf(angleStr, sizeof(angleStr), "%.1f", angleDeg);

    std::string gimbalModeStr;
    if (gimbalMode == 1) gimbalModeStr = "LEFT";
    else if (gimbalMode == 2) gimbalModeStr = "RIGHT";
    else gimbalModeStr = "BOTH";

    std::string targetInfo;
    if (hasTargetSite) {
        float distX = targetSite.centerX - state.x;
        float altToTarget = targetSite.yMean - state.y;
        targetInfo =
            "\n--- Landing Site ---"
            "\nTarget: (" + std::to_string((int)targetSite.centerX) + ", " + std::to_string((int)targetSite.yMean) + ")"
            "\nDist X: " + std::to_string((int)distX) + " px"
            "\nAlt: " + std::to_string((int)altToTarget);
    }

    float windMag = std::sqrt(wind.x * wind.x + wind.y * wind.y);
    char windStr[64];
    std::snprintf(windStr, sizeof(windStr), "(%.0f, %.0f) |W|=%.0f", wind.x, wind.y, windMag);

    std::string info = 
        "Status: " + status + "\n" +
        "Phase: " + std::string(phaseName ? phaseName : "?") + "\n" +
        "Mode: " + (autoMode ? "AUTOPILOT" : "MANUAL") + "\n" +
        "Angle: " + std::string(angleStr) + " deg\n" +
        "X: " + std::to_string((int)state.x) + "  Y: " + std::to_string((int)state.y) + "\n" +
        "Vx: " + std::to_string((int)state.vx) + "  Vy: " + std::to_string((int)state.vy) + "\n" +
        "Fuel: " + std::to_string((int)state.fuelMain) + "\n" +
        "Time: " + std::string(timeScaleStr) + "\n" +
        "Wind: " + std::string(windStr) +
        (autoMode ? "" : "\nGimbal: " + gimbalModeStr) +
        targetInfo;

    sf::Text txt(font);
    txt.setCharacterSize(14);
    txt.setFillColor(sf::Color::White);
    txt.setPosition({20.f, 20.f});
    txt.setString(info);
    window.draw(txt);

    // Индикатор ветра: круг + вектор
    {
        const float windMax = Config::WIND_MAX;
        const float r = 46.0f;
        sf::Vector2f center{ (float)Config::WINDOW_WIDTH - 70.0f, 70.0f };

        sf::CircleShape ring(r);
        ring.setOrigin({r, r});
        ring.setPosition(center);
        ring.setFillColor(sf::Color(0, 0, 0, 40));
        ring.setOutlineThickness(2.0f);
        ring.setOutlineColor(sf::Color(255, 255, 255, 120));
        window.draw(ring);

        sf::Vector2f v = wind;
        float L = std::sqrt(v.x * v.x + v.y * v.y);
        if (L > windMax && L > 1e-6f) {
            v.x = v.x / L * windMax;
            v.y = v.y / L * windMax;
            L = windMax;
        }

        sf::Vector2f end = center + sf::Vector2f(v.x / windMax * r, v.y / windMax * r);
        sf::VertexArray line(sf::PrimitiveType::Lines, 2);
        line[0].position = center;
        line[1].position = end;
        line[0].color = sf::Color(0, 220, 255, 200);
        line[1].color = sf::Color(0, 220, 255, 200);
        window.draw(line);

        
    }

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
}
