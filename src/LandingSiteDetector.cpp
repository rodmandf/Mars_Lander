#include "LandingSiteDetector.h"
#include <algorithm>
#include <cmath>

static float clampf(float v, float a, float b){ return std::max(a, std::min(v, b)); }

std::vector<LandingSite> detectLandingSites(const std::vector<RayHit>& hits,
                                            float roverX,
                                            const DetectorConfig& cfg)
{
    // реальные попадания
    std::vector<Vec2> pts;
    pts.reserve(hits.size());
    for (const auto& h : hits) {
        if (h.hit) pts.push_back(h.point);
    }
    if (pts.size() < 2) return {};

    
    std::sort(pts.begin(), pts.end(), [](const Vec2& a, const Vec2& b){ return a.x < b.x; });

    std::vector<LandingSite> out;

    size_t start = 0;
    while (start + 1 < pts.size()) {
        size_t end = start;

        float yMin = pts[start].y, yMax = pts[start].y;
        float ySum = pts[start].y;

        while (end + 1 < pts.size()) {
            Vec2 p0 = pts[end];
            Vec2 p1 = pts[end + 1];
            float dx = p1.x - p0.x;
            if (dx <= 1e-5f) { end++; continue; }

            // разрыв по X
            if (dx > cfg.maxGapX) break;

            float slope = (p1.y - p0.y) / dx;
            if (std::abs(slope) > cfg.maxSlope) break;

            end++;
            yMin = std::min(yMin, pts[end].y);
            yMax = std::max(yMax, pts[end].y);
            ySum += pts[end].y;

            if ((yMax - yMin) > cfg.maxBandY) break;
        }

        float x0 = pts[start].x;
        float x1 = pts[end].x;
        float lenX = x1 - x0;

        if (lenX >= cfg.minLenX) {
            LandingSite s;
            s.x0 = x0;
            s.x1 = x1;
            s.centerX = 0.5f * (x0 + x1);
            s.yMean = ySum / (float)(end - start + 1);
            s.slope = (pts[end].y - pts[start].y) / std::max(1e-5f, (pts[end].x - pts[start].x));

            // длина важна, дальность штрафуем
            float dist = std::abs(s.centerX - roverX);
            s.score = lenX - 0.25f * dist - 100.f * std::abs(s.slope);

            out.push_back(s);
        }

        start = end + 1;
    }

    std::sort(out.begin(), out.end(), [](const LandingSite& a, const LandingSite& b){
        return a.score > b.score;
    });

    return out;
}

bool pickBestSite(const std::vector<LandingSite>& sites, LandingSite& outBest) {
    if (sites.empty()) return false;
    outBest = sites.front();
    return true;
}
