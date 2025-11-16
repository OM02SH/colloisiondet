#include <iostream>
#include <vector>
#include <tuple>
#include "/home/omar_maher/vcpkg/buildtrees/tinyxml2/src/11.0.0-f00fa8a9d2.clean/tinyxml2.h"
#include <cmath>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <time.h>
#include <filesystem>
#include <chrono>
#include <string.h>
namespace fs = std::filesystem;
using namespace tinyxml2;

struct Lane {
    std::vector<std::pair<double,double>> leftBoundary;
    std::vector<std::pair<double,double>> rightBoundary;
};

struct ObstacleState {
    int id;
    double x, y;       // center
    int t;             // discrete time step
    double length, width;
    double orientation; // radians
};

struct ScenarioResult {
    std::string  filename;
    long double parseMs;
    long double computeMs;
    int violations;
    int collisions;
    int cars;
};

struct ScenarioData {
    std::map<int, std::vector<ObstacleState>> trajectories;
    std::vector<std::vector<std::pair<double,double>>> lanePolys;
};

std::atomic<int> violationsCount{0};
std::atomic<int> collisionCount{0};
struct Vec2 { double x, y; };
std::vector<std::vector<std::pair<double,double>>> lanePolys;
// key: obstacle ID, value: The obsracle which is essentially the vector of states over time
std::map<int, std::vector<ObstacleState>> trajectories;
std::atomic<bool> collisionFound{false};
std::mutex printMutex;
std::mutex resultsMutex;
std::vector<ScenarioResult> globalResults;

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

/// @brief Separating Axis Theorem for two oriented rectangles 
/// @param a,b rectangle obstacle
/// @return flag indicating overlap
static inline bool rectOverlap(const ObstacleState& a, const ObstacleState& b) {
    std::vector<Vec2> A = getCorners(a);
    std::vector<Vec2> B = getCorners(b);

    std::vector<Vec2> axes = {
        { A[1].x - A[0].x, A[1].y - A[0].y },   // A edge 1
        { A[3].x - A[0].x, A[3].y - A[0].y },   // A edge 2
        { B[1].x - B[0].x, B[1].y - B[0].y },   // B edge 1
        { B[3].x - B[0].x, B[3].y - B[0].y }    // B edge 2
    };

    for (auto axis : axes) {
        double len = std::sqrt(axis.x * axis.x + axis.y * axis.y);
        axis.x /= len;
        axis.y /= len;

        double minA = 1e300, maxA = -1e300;
        for (auto& p : A) {
            double proj = p.x * axis.x + p.y * axis.y;
            if (proj < minA) minA = proj;
            if (proj > maxA) maxA = proj;
        }

        double minB = 1e300, maxB = -1e300;
        for (auto& p : B) {
            double proj = p.x * axis.x + p.y * axis.y;
            if (proj < minB) minB = proj;
            if (proj > maxB) maxB = proj;
        }

        if (maxA < minB || maxB < minA)
            return false;
    }
    return true;
}

/// @brief Build lane polygon sorted from left then right boundaries
/// @param lane the lane
/// @return lane polygon
std::vector<std::pair<double,double>> buildLanePolygon(const Lane& lane) {
    std::vector<std::pair<double,double>> poly = lane.leftBoundary;
    for (auto it = lane.rightBoundary.rbegin(); it != lane.rightBoundary.rend(); ++it) 
        poly.push_back(*it); 
    return poly; 
}

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
        lanePolys.push_back(buildLanePolygon(lane));
    }

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
            trajectories[id].push_back(os);
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
            trajectories[id].push_back(os);
        }

        // Trajectory
        XMLElement* trajectory = dynObs->FirstChildElement("trajectory");
        if (!trajectory) continue;

        for (XMLElement* state = trajectory->FirstChildElement("state");
             state != nullptr;
             state = state->NextSiblingElement("state")) {

            XMLElement* pos = state->FirstChildElement("position")->FirstChildElement("point");
            double x = pos->FirstChildElement("x")->DoubleText();
            double y = pos->FirstChildElement("y")->DoubleText();

            int t = state->FirstChildElement("time")->FirstChildElement("exact")->IntText();

            double orientation = 0.0;
            if (XMLElement* orient = state->FirstChildElement("orientation")) {
                // orientation might be exact or interval; assume exact for simplicity
                if (XMLElement* exact = orient->FirstChildElement("exact")) {
                    orientation = exact->DoubleText();
                } else {
                    // fallback if only interval exists: use midpoint
                    XMLElement* interval = orient->FirstChildElement("interval");
                    if (interval) {
                        double start = interval->FirstChildElement("start")->DoubleText();
                        double end   = interval->FirstChildElement("end")->DoubleText();
                        orientation = 0.5 * (start + end);
                    }
                }
            }

            ObstacleState os{ id, x, y, t, length, width, orientation };
            trajectories[id].push_back(os);
        }
    }

    return true;
}

/// @brief check if point is on a segment of a polygon
/// @param x,y point to check coordinates
/// @param x1,y1 corner of segment coordinates
/// @param x2,y2 other corner of segment coordinates
/// @return flag indicating if point is on segment
static inline bool onSegment(double x, double y, double x1, double y1, double x2, double y2 ) {
    double cross = (x - x1)*(y2 - y1) - (y - y1)*(x2 - x1), eps = 1e-9;
    if (std::fabs(cross) > eps) return false;
    double dot = (x - x1)*(x2 - x1) + (y - y1)*(y2 - y1);
    if (dot < -eps) return false;
    double len2 = (x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1);
    if (dot - len2 > eps) return false;
    return true;
}

/// @brief check if point is inside or on polygon
/// @param x,y point to check coordinates 
/// @param poly polygon vertices
/// @return flag indicating if point is inside or on polygon
bool pointInPolygon(double x, double y, const std::vector<std::pair<double,double>>& poly) {
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) 
        if (onSegment(x, y, poly[j].first, poly[j].second, poly[i].first, poly[i].second)) 
            return true;
    bool inside = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        double xi = poly[i].first, yi = poly[i].second;
        double xj = poly[j].first, yj = poly[j].second;
        bool intersect = ((yi > y) != (yj > y)) &&
                         (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}

/*
/// @brief Find closest lane vertex to (x,y) and its opposite-side partner
/// @param x query x
/// @param y query y
/// @return pair: {closest point, opposite-side point}
std::pair<std::pair<double,double>, std::pair<double,double>> closestLanePointWithOpposite(double x, double y) {
   double bestDist2 = std::numeric_limits<double>::max();
    std::pair<double,double> bestPoint{0,0}, oppositePoint{0,0};
    for (const auto& poly : lanePolys) 
        for (size_t i = 0; i < poly.size(); ++i) {
            double dx = poly[i].first - x, dy = poly[i].second - y;
            double dist2 = dx*dx + dy*dy;
            if (dist2 < bestDist2) {
                bestDist2 = dist2;
                bestPoint = poly[i];
                size_t oppIdx = poly.size() - 1 - i;
                oppositePoint = poly[oppIdx];
            }
        }
    return {bestPoint, oppositePoint};
}*/

/// @brief Check if each corner of the Obstcal is inside some lane polygon
/// @param Obstacle the obstacle 
/// @return flag indicating if Obstical is inside road
bool isInsideRoad(const ObstacleState& Obstacle, const std::vector<std::vector<std::pair<double,double>>>& lanePolys) {
    auto corners = getCorners(Obstacle);
    for (const auto& c : corners) {
        bool insideAnyLane = false;
        for (const auto& poly : lanePolys) 
            if (pointInPolygon(c.x, c.y, poly)) {
                insideAnyLane = true;
                break;
            }
        if (!insideAnyLane){
            //std::pair<std::pair<double,double>, std::pair<double,double>> q =
            //    closestLanePointWithOpposite(c.x, c.y);
            //std::lock_guard<std::mutex> lock(printMutex);
            //std::cout << "Obstacle " << Obstacle.id << " corner at (" << c.x << ", " << c.y << ") "
            //    << "closest Lane Point" << "(" << q.first.first << ", " << q.first.second << ") " << 
            //    "opposide" << "(" << q.second.first << ", " << q.second.second << ") "
            //    << "time" << Obstacle.t << std::endl;
            return false;
        }
    }
    return true;
}

/// @brief Thread worker function for checking collisions for a specific obstacle
/// @param Id obstacle ID
void workerForObstacle(int Id){
    const auto& myTraj = trajectories.at(Id);

    std::vector<int> otherIds;
    otherIds.reserve(trajectories.size() - 1);
    for (const auto& kv : trajectories) {
        if (kv.first > Id) otherIds.push_back(kv.first);
    }

    for (const ObstacleState& s : myTraj) {
        //if (collisionFound.load(std::memory_order_relaxed)) return;

        // Road containment check
        if (!isInsideRoad(s,lanePolys) && s.t != 0) {
            //std::lock_guard<std::mutex> lock(printMutex);
            //std::cout << "Obstacle " << s.id << " exits the road at time " << s.t << std::endl;
            violationsCount.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        // Collision check against other obstacles
        for (int oid : otherIds) {
            //if (collisionFound.load(std::memory_order_relaxed)) return;

            const  std::vector<ObstacleState>& oTraj = trajectories.at(oid);
            auto it = std::lower_bound(oTraj.begin(), oTraj.end(), s.t,
                [](const ObstacleState& st, int tt){ return st.t < tt; });
            if (it != oTraj.end() && it->t == s.t) {
                if (rectOverlap(s, *it)) {
                    std::lock_guard<std::mutex> lock(printMutex);
                    std::cout << "collision at time " << s.t
                              << " between obstacle " << s.id
                              << " and obstacle " << it->id << std::endl;
                    collisionCount.fetch_add(1, std::memory_order_relaxed);
                    return;
                }
            }
        }
    }
}

int main() {
    long double totalMilliseconds = 0.0, totalcars = 0.0;
    int c= 0, v=0, n=0;
    for (const auto& entry : fs::directory_iterator("/home/omar_maher/commonroadRl/FIX_commonroad_rl-master/commonroad_rl/tutorials/data/inD-dataset-v1.0/xmls")) {
        std::cout << "Processing file: " << entry.path().filename() << std::endl;
        auto startTime = std::chrono::high_resolution_clock::now();

        if (entry.is_regular_file() && entry.path().extension() == ".xml") {
            if (!parseScenario(entry.path().c_str()))
                return -1;
        }
        else continue;
        totalcars += trajectories.size();
        auto parseEndTime = std::chrono::high_resolution_clock::now();
        std::vector<std::thread> threads;
        threads.reserve(trajectories.size());
        for (const auto& kv : trajectories) 
            threads.emplace_back(workerForObstacle, kv.first);

        for (auto& th : threads) th.join();
        auto endTime = std::chrono::high_resolution_clock::now();
        v += violationsCount.load();
        c += collisionCount.load();
        n += 1;
        std::cout << "Violations count: " << violationsCount.load() << std::endl;
        std::cout << "Collisions count: " << collisionCount.load() << std::endl;
        std::cout << "Parsing time: " << std::chrono::duration_cast<std::chrono::milliseconds>(parseEndTime - startTime).count() << "." << std::chrono::duration_cast<std::chrono::microseconds>(parseEndTime - startTime).count() << " ms" << std::endl;
        std::cout << "Computation time: " << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - parseEndTime).count() << "." << std::chrono::duration_cast<std::chrono::microseconds>(endTime - parseEndTime).count() << " ms" << std::endl;
        std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() << "." << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << " ms" << std::endl << std::endl;

        totalMilliseconds += std::chrono::duration_cast<std::chrono::milliseconds>(endTime - parseEndTime).count() * 1.0 + (1.0 *std::chrono::duration_cast<std::chrono::microseconds>(endTime - parseEndTime).count()) / 1000;
        collisionFound.store(false);
        trajectories.clear();
        lanePolys.clear();      
        violationsCount.store(0);  
        collisionCount.store(0);
    }
    std::cout << "Total time: " << totalMilliseconds << " ms" << std::endl;
    std::cout << "Average time per car: " << (totalMilliseconds * 1000 / totalcars) << " microSec" << std::endl;
    std::cout << "Average violations per scenario: " << (1.0 * v / n) << std::endl;
    std::cout << "Average collisions per scenario: " << (1.0 * c / n) << std::endl;
    std::cout << "Total scenarios processed: " << n << std::endl;
    return 0;
}
