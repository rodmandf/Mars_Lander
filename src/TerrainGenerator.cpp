#include "TerrainGenerator.h"
#include <cmath>
#include <cstdlib>
#include <algorithm>

float TerrainGenerator::getNoise(float x, int seed) {
    return std::sin(x * 0.01f + seed) * 120.0f + 
           std::sin(x * 0.02f + seed * 3) * 60.0f +
           std::sin(x * 0.05f) * 15.0f; 
}

std::vector<float> TerrainGenerator::generate(int width, int seed) {
    std::vector<float> terrain(width);
    std::srand(seed);

    auto clampf = [](float v, float a, float b) { return std::max(a, std::min(v, b)); };
    auto lerp   = [](float a, float b, float t) { return a + (b - a) * t; };
    auto smoothstep01 = [&](float t) {
        t = clampf(t, 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t); 
    };


    const float baseY = Config::WINDOW_HEIGHT - 150.0f; 
    const float amp = 1.0f; 

    for (int x = 0; x < width; ++x) {
        float noiseVal = getNoise((float)x, seed); 
        terrain[x] = baseY - amp * noiseVal; 
        terrain[x] = clampf(terrain[x], 80.0f, (float)Config::WINDOW_HEIGHT - 20.0f);
    }

    std::vector<float> tmp = terrain;
    for (int pass = 0; pass < 3; ++pass) {
        tmp[0] = terrain[0];
        tmp[width - 1] = terrain[width - 1];
        for (int x = 1; x < width - 1; ++x) {
            tmp[x] = (terrain[x - 1] + 2.0f * terrain[x] + terrain[x + 1]) * 0.25f;
        }
        terrain.swap(tmp);
    }

    int numZones = 1 + std::rand() % 4; 
    const int zoneWidth = 60; 
    const int blendW = 30; 

    std::vector<int> usedCenters;

    for (int i = 0; i < numZones; ++i) {
        int zoneX = 80 + std::rand() % (width - 160);
        bool ok = true;
        for (int c : usedCenters) {
            if (std::abs(c - zoneX) < zoneWidth + 2 * blendW) { ok = false; break; }
        }
        if (!ok) { --i; continue; }
        usedCenters.push_back(zoneX);

        float zoneHeight = terrain[zoneX];

        int padL = zoneX - zoneWidth / 2;
        int padR = zoneX + zoneWidth / 2;

        // flat core
        for (int x = padL; x <= padR; ++x) {
            if (x >= 0 && x < width) terrain[x] = zoneHeight;
        }

        // left blend
        for (int x = padL - blendW; x < padL; ++x) {
            if (x <= 0 || x >= width) continue;
            float t = (float)(x - (padL - blendW)) / (float)blendW; 
            float w = smoothstep01(t);
            terrain[x] = lerp(terrain[x], zoneHeight, w);
        }

        // right blend
        for (int x = padR + 1; x <= padR + blendW; ++x) {
            if (x <= 0 || x >= width) continue;
            float t = (float)(x - (padR + 1)) / (float)blendW; 
            float w = smoothstep01(t);
            terrain[x] = lerp(zoneHeight, terrain[x], w);
        }
    }

    return terrain;
}

std::vector<int> TerrainGenerator::findLandingZones(const std::vector<float>& terrain) {
    std::vector<int> zones;
    int currentFlat = 0;
    
    for (size_t x = 1; x < terrain.size(); x++) {
        if (std::abs(terrain[x] - terrain[x-1]) < 0.1f) {
            currentFlat++;
        } else {
            if (currentFlat > 40) { // Если нашли зону > 40 пикселей
                zones.push_back(x - (currentFlat / 2));
            }
            currentFlat = 0;
        }
    }
    // Проверка последнего участка
    if (currentFlat > 40) zones.push_back(terrain.size() - (currentFlat / 2));
    
    return zones;
}
