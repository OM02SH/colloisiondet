"""
Collision Detection for Commonroad Scenarios 
Author: [Omar Shaat/OM02SH]
Email: omar.shaat@tum.de
"""
    
#include <iostream>
#include <vector>
#include <utility>
#include <cmath>
#include "tinyxml2.h"
#include "vcpkg/packages/tbb_x64-linux/include/tbb/parallel_for.h"
#include "vcpkg/packages/tbb_x64-linux/include/tbb/blocked_range.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <immintrin.h>

namespace bg = boost::geometry;

using Point  = bg::model::d2::point_xy<double>;
using Poly   = bg::model::polygon<Point>;
using Multi  = bg::model::multi_polygon<Poly>;

using namespace tinyxml2;

struct Vec2 { double x, y; };

struct Lane {
    std::vector<Vec2> leftBoundary;
    std::vector<Vec2> rightBoundary;
};

struct ObstacleState {
    int id;
    double x, y;       
    int t;             
    double length, width;
    double orientation;
};

struct PolyCache {
    std::vector<double> xi, yi, xj, yj;  
    std::vector<double> dx, dy;          // NEW: edge direction cached
    std::vector<double> xDivy;           // already used in point-in-poly
    size_t n;

    PolyCache(size_t k) : n(k) {
        xi.resize(n); yi.resize(n);
        xj.resize(n); yj.resize(n);
        dx.resize(n); dy.resize(n);
        xDivy.resize(n);
    }
};

struct RoadUnion {
    std::vector<Vec2> outer;
    std::vector<std::vector<Vec2>> holes;
};

struct Road {
    std::vector<Vec2> outer;
    std::vector<std::vector<Vec2>> holes;
};

std::atomic<int> violationsCount{0};
std::vector<Lane> lanes;
std::map<int, ObstacleState> positions;
PolyCache roadCache(0);
std::vector<PolyCache> holeCaches;

/// @brief parse scenario XML file
/// @param filename 
/// @param data scenario data to fill
/// @return flag indicating success
bool parseScenario(const char* filename) {
    XMLDocument doc;
    if (doc.LoadFile(filename) != XML_SUCCESS) {
        std::cerr << "Error loading XML file: " << filename << std::endl;
        return false;
    }
    XMLElement* root = doc.FirstChildElement("commonRoad");
    if (!root) {
        std::cerr << "No <commonRoad> root element." << std::endl;
        return false;
    }

    for (XMLElement* lanelet = root->FirstChildElement("lanelet");
         lanelet != nullptr;
         lanelet = lanelet->NextSiblingElement("lanelet")) {
        Lane lane;
        XMLElement* leftBound = lanelet->FirstChildElement("leftBound");
        if (leftBound) {
            for (XMLElement* point = leftBound->FirstChildElement("point");
                 point != nullptr;
                 point = point->NextSiblingElement("point")) {
                double x = point->FirstChildElement("x")->DoubleText();
                double y = point->FirstChildElement("y")->DoubleText();
                lane.leftBoundary.push_back({x,y});
            }
        }
        XMLElement* rightBound = lanelet->FirstChildElement("rightBound");
        if (rightBound) {
            for (XMLElement* point = rightBound->FirstChildElement("point");
                 point != nullptr;
                 point = point->NextSiblingElement("point")) {
                double x = point->FirstChildElement("x")->DoubleText();
                double y = point->FirstChildElement("y")->DoubleText();
                lane.rightBoundary.push_back({x,y});
            }
        }
        lanes.push_back(lane);
    }
    int c = 0;
    for (XMLElement* dynObs = root->FirstChildElement("staticObstacle");
         dynObs != nullptr;
         dynObs = dynObs->NextSiblingElement("staticObstacle")) {

        int id = dynObs->IntAttribute("id");

        // Shape
        double length = 0.0, width = 0.0;
        if (XMLElement* shape = dynObs->FirstChildElement("obstacleShape")) {
            if (XMLElement* rect = shape->FirstChildElement("rectangle")) {
                if (XMLElement* le = rect->FirstChildElement("length")) length = le->DoubleText();
                if (XMLElement* wi = rect->FirstChildElement("width"))  width  = wi->DoubleText();
            }
        } else if (XMLElement* shape2 = dynObs->FirstChildElement("shape")) { 
            if (XMLElement* rect = shape2->FirstChildElement("rectangle")) {
                if (XMLElement* le = rect->FirstChildElement("length")) length = le->DoubleText();
                if (XMLElement* wi = rect->FirstChildElement("width"))  width  = wi->DoubleText();
            }
        }

        if (XMLElement* init = dynObs->FirstChildElement("initialState")) {
            double x = 0.0, y = 0.0, orientation = 0.0;
            int t = 0;
        
            if (XMLElement* pos = init->FirstChildElement("position")) {
                if (XMLElement* pt = pos->FirstChildElement("point")) {
                    x = pt->FirstChildElement("x")->DoubleText();
                    y = pt->FirstChildElement("y")->DoubleText();
                }
            }
            if (XMLElement* tim = init->FirstChildElement("time")) {
                if (XMLElement* ex = tim->FirstChildElement("exact")) t = ex->IntText();
            }
            if (XMLElement* orient = init->FirstChildElement("orientation")) {
                if (XMLElement* ex = orient->FirstChildElement("exact")) orientation = ex->DoubleText();
                else if (XMLElement* interval = orient->FirstChildElement("interval")) {
                    double start = interval->FirstChildElement("start")->DoubleText();
                    double end   = interval->FirstChildElement("end")->DoubleText();
                    orientation = 0.5 * (start + end);
                }
            }
        
            ObstacleState os{ id, x, y, t, length, width, orientation };
            positions[c++] = (os);
        }
    }

    for (XMLElement* dynObs = root->FirstChildElement("dynamicObstacle");
         dynObs != nullptr;
         dynObs = dynObs->NextSiblingElement("dynamicObstacle")) {

        int id = dynObs->IntAttribute("id");

        // Shape
        double length = 0.0, width = 0.0;
        if (XMLElement* shape = dynObs->FirstChildElement("obstacleShape")) {
            if (XMLElement* rect = shape->FirstChildElement("rectangle")) {
                if (XMLElement* le = rect->FirstChildElement("length")) length = le->DoubleText();
                if (XMLElement* wi = rect->FirstChildElement("width"))  width  = wi->DoubleText();
            }
        } else if (XMLElement* shape2 = dynObs->FirstChildElement("shape")) { 
            if (XMLElement* rect = shape2->FirstChildElement("rectangle")) {
                if (XMLElement* le = rect->FirstChildElement("length")) length = le->DoubleText();
                if (XMLElement* wi = rect->FirstChildElement("width"))  width  = wi->DoubleText();
            }
        }

        if (XMLElement* init = dynObs->FirstChildElement("initialState")) {
            double x = 0.0, y = 0.0, orientation = 0.0;
            int t = 0;
            if (XMLElement* pos = init->FirstChildElement("position")) 
                if (XMLElement* pt = pos->FirstChildElement("point")) {
                    x = pt->FirstChildElement("x")->DoubleText();
                    y = pt->FirstChildElement("y")->DoubleText();
                }
            if (XMLElement* tim = init->FirstChildElement("time")) 
                if (XMLElement* ex = tim->FirstChildElement("exact")) t = ex->IntText();
            if (XMLElement* orient = init->FirstChildElement("orientation")) {
                if (XMLElement* ex = orient->FirstChildElement("exact")) orientation = ex->DoubleText();
                else if (XMLElement* interval = orient->FirstChildElement("interval")) {
                    double start = interval->FirstChildElement("start")->DoubleText();
                    double end   = interval->FirstChildElement("end")->DoubleText();
                    orientation = 0.5 * (start + end);
                }
            }
            ObstacleState os{ id, x, y, t, length, width, orientation };
            positions[c++] = (os);
        }
    }

    return true;
}

/// @brief convert struct Lane to bg::model::polygon
Poly laneToPolygon(const Lane& lane) {
    Poly poly;
    for (auto &p : lane.leftBoundary)
        bg::append(poly.outer(), Point(p.x, p.y));
    for (auto it = lane.rightBoundary.rbegin(); it != lane.rightBoundary.rend(); ++it)
        bg::append(poly.outer(), Point(it->x, it->y));
    bg::correct(poly);
    return poly;
}

/// @brief add buffer of 5cm to the lane for small errors in given scenarios 
/// @param poly 
/// @return buffered lane
Poly bufferLane(const Poly& poly)
{
    Multi result;
    bg::strategy::buffer::distance_symmetric<double> distance(0.05);
    bg::strategy::buffer::join_round join;
    bg::strategy::buffer::end_round end;
    bg::strategy::buffer::point_circle circle(8);
    bg::strategy::buffer::side_straight side;
    bg::buffer(poly, result, distance, side, join, end, circle);
    return result.front();
}

/// @brief convert the lanes vector to struct Road with outer and holes
Road buildRoad() {
    Multi acc;
    for (const Lane& ln : lanes) {
        Poly p  = laneToPolygon(ln);
        Poly bp = bufferLane(p);
        Multi temp;
        bg::union_(acc, bp, temp);
        acc = temp;
    }
    if (acc.empty()) return {};
    Poly road = acc.front();
    Road result;
    for (auto& pt : road.outer())
        result.outer.emplace_back(pt.x(), pt.y());
    for (auto& inner : road.inners()) {
        std::vector<Vec2> hole;
        for (auto& pt : inner)
            hole.emplace_back(pt.x(), pt.y());
        result.holes.push_back(std::move(hole));
    }
    return result;
}

/// @brief convert from std::vector<Vec2> to PolyCache
/// @param poly 
PolyCache preprocessPolygon(const std::vector<Vec2>& poly) {
    PolyCache c(poly.size());
    for (size_t i = 0; i < c.n; ++i) {
        size_t j = (i + c.n - 1) % c.n;
        c.xi[i] = poly[i].x; c.yi[i] = poly[i].y;
        c.xj[i] = poly[j].x; c.yj[i] = poly[j].y;
        c.dx[i] = c.xj[i] - c.xi[i];
        c.dy[i] = c.yj[i] - c.yi[i];
        if(c.yj[i] == c.yi[i])
            c.xDivy[i] = 0.0; // horizontal edge nothing more is needed as the check (c.yi[i] > p.y) != (c.yj[i] > p.y) will be false
        else 
            c.xDivy[i] = (c.xj[i] - c.xi[i]) / (c.yj[i] - c.yi[i]);
    }
    return c;
}

/// @brief Check if point is on the edge of the polygon
/// @param p point to check
/// @param c preprocessed polygon cache
/// @return flag indicating if point is on edge
inline bool onEdge(const Vec2& p, const PolyCache& c) {
    for (size_t i = 0; i < c.n; ++i) {
        double t = c.dx[i] ? (p.x - c.xi[i]) / c.dx[i] : (p.y - c.yi[i]) / c.dy[i];
        if (t >= 0 && t <= 1 && std::abs(c.xi[i] + t*c.dx[i] - p.x) < 1e-9 &&
            std::abs(c.yi[i] + t * c.dy[i] - p.y) < 1e-9)
            return true;
    }
    return false;
}

/// @brief Check if all points are inside polygon using cached data
/// @param points points to check
/// @param c preprocessed polygon cache
/// @return flag indicating if all points are inside polygon
inline bool pointCheck(const Vec2& p, const PolyCache& c) {
    bool inside = false;
    for (size_t i = 0; i < c.n; ++i) 
        if (((c.yi[i] > p.y) != (c.yj[i] > p.y)) && (p.x <  (p.y - c.yi[i]) * c.xDivy[i] + c.xi[i]))
            inside = !inside;
    return !(!inside && !onEdge(p, c));
}

/// @brief checks if the sides between corners p1 and p2 intersects the hole c
/// @param p1,p2 
/// @param hole
inline bool segmentCheck(const Vec2& p1, const Vec2& p2, const PolyCache& hole) {
    double sx = p2.x - p1.x, sy = p2.y - p1.y;
    for (size_t i = 0; i < hole.n; ++i) {
        double denom = sx * hole.dy[i] - sy * hole.dx[i];
        if (std::abs(denom) < 1e-12) continue;
        double u = ((hole.xi[i] - p1.x) * hole.dy[i] - (hole.yi[i] - p1.y) * hole.dx[i]) / denom;
        if (u < 0 || u > 1) continue;
        double v = ((hole.xi[i] - p1.x) * sy - (hole.yi[i] - p1.y) * sx) / denom;
        if (v < 0 || v > 1) continue;
        return true;
    }
    return false;
}

/// @brief checks if the sides between corners p1 and p2 intersects the outer union of the road
/// @param p1,p2 
/// @param c road union
inline bool segmentExitsPolygon(const Vec2& p1, const Vec2& p2, const PolyCache& c) {
#if defined(__AVX2__)
    const int N = c.n;

    // Broadcast values for segment
    __m256 sxv = _mm256_set1_ps((float)(p2.x - p1.x));
    __m256 syv = _mm256_set1_ps((float)(p2.y - p1.y));
    __m256 pxv = _mm256_set1_ps((float)p1.x);
    __m256 pyv = _mm256_set1_ps((float)p1.y);

    for (int i = 0; i < N; i += 8)
    {
        // Load polygon data
        __m256 xi = _mm256_loadu_ps(reinterpret_cast<const float*>(&c.xi[i]));
        __m256 yi = _mm256_loadu_ps(reinterpret_cast<const float*>(&c.yi[i]));
        __m256 dx = _mm256_loadu_ps(reinterpret_cast<const float*>(&c.dx[i]));
        __m256 dy = _mm256_loadu_ps(reinterpret_cast<const float*>(&c.dy[i]));

        // denom = sx*ey − sy*ex
        __m256 denom = _mm256_fmsub_ps(sxv, dy, _mm256_mul_ps(syv, dx));

        // |denom| > eps?
        __m256 absDen = _mm256_andnot_ps(_mm256_set1_ps(-0.f), denom); 
        __m256 maskDen = _mm256_cmp_ps(absDen, _mm256_set1_ps(1e-12f), _CMP_GT_OQ);

        if (_mm256_movemask_ps(maskDen) == 0)
            continue; // all edges parallel → skip

        // rx = xi − p1.x ; ry = yi − p1.y
        __m256 rx = _mm256_sub_ps(xi, pxv);
        __m256 ry = _mm256_sub_ps(yi, pyv);

        // u = (rx*ey − ry*ex) / denom
        __m256 u = _mm256_div_ps(
            _mm256_fmsub_ps(rx, dy, _mm256_mul_ps(ry, dx)),
            denom
        );

        // Check 0 < u < 1
        __m256 maskU =
            _mm256_and_ps(
                _mm256_cmp_ps(u, _mm256_set1_ps(0.0f), _CMP_GT_OQ),
                _mm256_cmp_ps(u, _mm256_set1_ps(1.0f), _CMP_LT_OQ)
            );

        if (_mm256_movemask_ps(maskU) == 0)
            continue;

        // v = (rx*sy − ry*sx) / denom
        __m256 v = _mm256_div_ps(
            _mm256_fmsub_ps(rx, syv, _mm256_mul_ps(ry, sxv)),
            denom
        );

        // Check 0 < v < 1
        __m256 maskV =
            _mm256_and_ps(
                _mm256_cmp_ps(v, _mm256_set1_ps(0.0f), _CMP_GT_OQ),
                _mm256_cmp_ps(v, _mm256_set1_ps(1.0f), _CMP_LT_OQ)
            );

        // Intersection exists if *both* masks have at least one bit
        __m256 both = _mm256_and_ps(maskU, maskV);
        if (_mm256_movemask_ps(both))
            return true;
    }

    return false;

#else
    // Scalar fallback (your original implementation)
    const double sx = p2.x - p1.x, sy = p2.y - p1.y;
    for (size_t i = 0; i < c.n; ++i) {
        double ex = c.dx[i], ey = c.dy[i];
        double denom = sx * ey - sy * ex;
        double rx = c.xi[i] - p1.x;
        double ry = c.yi[i] - p1.y;
        double u = (rx * ey - ry * ex) / denom;
        if (u <= 0.0 || u >= 1.0) continue;
        double v = (rx * sy - ry * sx) / denom;
        if (v <= 0.0 || v >= 1.0) continue;

        return true;
    }
    return false;
#endif
}

/// @brief cheking the side for outer union and holes overlap
/// @param points rectangle corners
inline bool segmentsIntersectPolygon(const std::vector<Vec2>& points) {
    for (size_t i = 0; i < points.size(); ++i) {
        const Vec2& p = points[(i + 1) % points.size()];
        if (segmentExitsPolygon(points[i], p, roadCache)) return true;
    }
    for (const PolyCache& c : holeCaches){
        for (size_t i = 0; i < points.size(); ++i) {
            const Vec2& p1 = points[i];
            const Vec2& p2 = points[(i + 1) % points.size()];
            if (segmentCheck(p1, p2, c)) return true;
        }
    }
    return false;
}

/// @brief get the 4 corners of an oriented rectangle obstacle
/// @param obs the obstacle
/// @return vector of corner points
static inline std::vector<Vec2> getCorners(const ObstacleState& obs) {
    double hl = obs.length / 2.0, hw = obs.width / 2.0, c = std::cos(obs.orientation), s = std::sin(obs.orientation);
    std::vector<Vec2> local = {{ hl,  hw},  {-hl,  hw},  {-hl, -hw},  { hl, -hw}}, global;
    global.reserve(4);
    for (auto& p : local) {
        Vec2 r{ p.x * c - p.y * s, p.x * s + p.y * c };
        global.push_back({ obs.x + r.x, obs.y + r.y });
    }
    return global;
}

/// @brief Check if each corner of the Obstcal is inside some lane polygon
/// @param Obstacle the obstacle 
/// @return flag indicating if Obstical is inside road
bool isInsideRoad(const ObstacleState& Obstacle) {
    std::vector<Vec2> corners = getCorners(Obstacle);
    for (const Vec2& c : corners) 
        if (pointCheck(c, roadCache)){
            for(auto& h : holeCaches)
                if(pointCheck(c,h))
                    return false;
        }
        else return false;
    return !segmentsIntersectPolygon(corners);
}

/// @brief Thread worker function for checking collisions for a specific obstacle
/// @param Id obstacle ID
void workerForObstacle(int Id){
    const ObstacleState& position = positions.at(Id);
    if (!isInsideRoad(position)) 
        violationsCount.fetch_add(1, std::memory_order_relaxed);
}

int main(){
    auto startTime = std::chrono::high_resolution_clock::now(); 
    if(!parseScenario("ZAM/ZAM_Merge-1_1_I-1-1.cr.xml")) return -1;
    Road r = buildRoad();
    roadCache = preprocessPolygon(r.outer);
    for(const auto& hole: r.holes)
        holeCaches.push_back(preprocessPolygon(hole));
    auto parseEndTime = std::chrono::high_resolution_clock::now();
    tbb::parallel_for(
        tbb::blocked_range<int>(0, 1'000'000),
        [](const tbb::blocked_range<int>& r) {
            for (int i = r.begin(); i != r.end(); ++i) {
                int pos_index = i % positions.size();  
                workerForObstacle(pos_index);
            }
        }
    );
    auto endTime = std::chrono::high_resolution_clock::now();
    std::cout << "Violations count: " << violationsCount.load() << std::endl;
    auto parseDuration = (parseEndTime - startTime);
    auto computeDuration = (endTime - parseEndTime);
    auto totalDuration = (endTime - startTime);

    double parseMs = std::chrono::duration<double, std::milli>(parseDuration).count();
    double computeMs = std::chrono::duration<double, std::milli>(computeDuration).count();
    double totalMs = std::chrono::duration<double, std::milli>(totalDuration).count();

    std::cout << "Parsing time: " << parseMs << " ms\n";
    std::cout << "Computation time: " << computeMs << " ms\n";
    std::cout << "Total time: " << totalMs << " ms\n";

    return 0;
}
