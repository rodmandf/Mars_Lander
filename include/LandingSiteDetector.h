#pragma once
#include <vector>
#include "RadarTypes.h"
#include "Config.h"

struct LandingSite {
    float x0 = 0.f;
    float x1 = 0.f;
    float centerX = 0.f;
    float yMean = 0.f;
    float slope = 0.f;
    float score = 0.f;
};

struct DetectorConfig {
    float maxGapX = 10.f;       // максимальный разрыв между точками
    float minLenX = 40.f;       // минимальная длина площадки
    float maxSlope = 0.05f;     // ограничение на кривизну поверхности
    float maxBandY = std::tan(Config::MAX_LANDING_ANGLE_RAD);      // ограничение на угол наклона площадки
};

std::vector<LandingSite> detectLandingSites(const std::vector<RayHit>& hits,
                                            float roverX,
                                            const DetectorConfig& cfg);

bool pickBestSite(const std::vector<LandingSite>& sites, LandingSite& outBest);
