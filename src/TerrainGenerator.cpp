#include "TerrainGenerator.h"
#include "FastNoiseLite.h"
#include <cstdlib>
#include <algorithm>
#include <cmath>

std::vector<float> TerrainGenerator::generate(int width, int seed) {
    std::vector<float> terrain(width);
    std::srand(seed);

    auto clampf = [](float v, float a, float b) { return std::max(a, std::min(v, b)); };
    auto lerp   = [](float a, float b, float t) { return a + (b - a) * t; };
    auto smoothstep01 = [&](float t) {
        t = clampf(t, 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t); 
    };


    // Шум FastNoiseLite
    FastNoiseLite noise;
    noise.SetSeed(seed);
    noise.SetNoiseType(FastNoiseLite::NoiseType_OpenSimplex2);
    noise.SetFractalType(FastNoiseLite::FractalType_FBm);
    noise.SetFractalOctaves(3);
    noise.SetFractalLacunarity(2.0f);
    noise.SetFractalGain(0.4f);
    noise.SetFrequency(0.002f);

    const float baseY = Config::WINDOW_HEIGHT * 0.75f;
    const float mountainScale = 140.0f;

    for (int x = 0; x < width; ++x) {
        float n = noise.GetNoise((float)x, 0.0f);

        // Основной рельеф
        terrain[x] = baseY - (n * mountainScale);

        float pebbles = noise.GetNoise((float)x * 15.0f, 100.0f) * 1.5f;
        terrain[x] += pebbles;

        terrain[x] = clampf(terrain[x], 100.0f, (float)Config::WINDOW_HEIGHT - 20.0f);
    }

    // Небольшое сглаживание
    {
        std::vector<float> tmp = terrain;
        int smoothRadius = 2;
        for (int pass = 0; pass < 1; ++pass) {
            tmp = terrain;
            for (int x = smoothRadius; x < width - smoothRadius; ++x) {
                float sum = 0.0f;
                for (int k = -smoothRadius; k <= smoothRadius; ++k) sum += tmp[x + k];
                terrain[x] = sum / (2.0f * smoothRadius + 1.0f);
            }
        }
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

        for (int x = padL; x <= padR; ++x) {
            if (x >= 0 && x < width) terrain[x] = zoneHeight;
        }

        for (int x = padL - blendW; x < padL; ++x) {
            if (x <= 0 || x >= width) continue;
            float t = (float)(x - (padL - blendW)) / (float)blendW; 
            float w = smoothstep01(t);
            terrain[x] = lerp(terrain[x], zoneHeight, w);
        }

        for (int x = padR + 1; x <= padR + blendW; ++x) {
            if (x <= 0 || x >= width) continue;
            float t = (float)(x - (padR + 1)) / (float)blendW; 
            float w = smoothstep01(t);
            terrain[x] = lerp(zoneHeight, terrain[x], w);
        }
    }

    // Сильное сглаживание
    {
        std::vector<float> tmp = terrain;
        int smoothRadius = 6;
        for (int pass = 0; pass < 2; ++pass) {
            tmp = terrain;
            for (int x = smoothRadius; x < width - smoothRadius; ++x) {
                float sum = 0.0f;
                for (int k = -smoothRadius; k <= smoothRadius; ++k) sum += tmp[x + k];
                terrain[x] = sum / (2.0f * smoothRadius + 1.0f);
            }
        }
    }

    return terrain;
}
