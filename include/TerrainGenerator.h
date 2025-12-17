#pragma once
#include "Config.h"
#include <vector>

class TerrainGenerator {
public:
    std::vector<float> generate(int width, int seed);
    std::vector<int> findLandingZones(const std::vector<float>& terrain);

private:
    float getNoise(float x, int seed);
};
