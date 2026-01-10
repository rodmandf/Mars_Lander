#pragma once
#include "Config.h"
#include <vector>

class TerrainGenerator {
public:
    std::vector<float> generate(int width, int seed);

private:
    float getNoise(float x, int seed);
};
