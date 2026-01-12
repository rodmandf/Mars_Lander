#pragma once
#include "Config.h"
#include <vector>

class TerrainGenerator {
public:
    std::vector<float> generate(int width, int seed);
};
