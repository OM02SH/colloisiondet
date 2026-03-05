# ⚡ High-Performance Collision Detection for CommonRoad
**C++23 | SIMD-Accelerated | Multi-Threaded**

This project implements an ultra-fast collision detection engine designed to validate obstacle orientations against road boundaries in **CommonRoad 2020a** XML scenarios. By leveraging hardware acceleration and modern C++ concurrency, it achieves sub-100ms performance for massive query loads.

---

## 📊 Performance Benchmark

* **Scenario:** `ZAM_Merge-1_1_I-1-1`
* **Obstacles:** 20 distinct positions/orientations of the same vehicle.
* **Workload:** **1,000,000** synthetic collision queries.
* **Result:** **~85 ms** total execution time (averaged over 1,000 runs).

---

## 🛠 Technical Stack

| Category | Technology |
| :--- | :--- |
| **Language** | **C++23** |
| **Concurrency** | **Intel OneTBB** (`parallel_for`) |
| **Geometry** | **Boost.Geometry** |
| **XML Parsing** | **tinyxml2** |
| **Hardware Accel** | **AVX2 SIMD** (`immintrin.h`) |

---

## 🚀 Methodology

The detection pipeline consists of a robust preprocessing stage followed by a high-speed parallel evaluation loop.

### 1. Map Preprocessing
* **Lane Reconstruction:** Converts individual lanes into clockwise polygons by concatenating the left boundary with the reversed right boundary.
* **Symmetric Buffering:** Applies a `0.05m` buffer via `boost::geometry::buffer` to regularize shapes and ensure robust union operations.
* **Road Union:** Merges all buffered lane polygons into a single "Road Outline." The largest resulting polygon (including holes) is designated as the valid driving surface.

### 2. Optimization & Caching
To maximize throughput, the engine pre-computes a **Polygon Cache** for every edge:
* Vertex coordinates ($x_i, y_i, x_j, y_j$) and edge directions ($dx, dy$).
* Slope terms ($x/y$) to accelerate the **Winding-Test** (Point-in-Polygon).



### 3. Collision Verification Logic
For every obstacle state, the following hierarchy of checks is performed:
1.  **Containment:** All four corners of the vehicle rectangle must reside within the outer road polygon.
2.  **Exclusion:** No corner may reside inside a "hole" polygon (e.g., center islands).
3.  **Boundary Intersection:** No rectangle edge may exit the outer polygon or intersect any hole boundary.



---

## 💡 Implementation Highlights

* **SIMD Acceleration:** Segment-intersection tests are vectorized using **AVX2**, significantly reducing branching overhead during edge-heavy checks.
* **Thread-Safe Accounting:** Violations are tracked using relaxed atomic increments to minimize contention across CPU cores.
* **Boundary Logic:** In this implementation, being exactly on the road boundary is considered a "safe" state.

---

## 🔗 References & Credits

* **Geometry Concepts:** [Line Intersection and its Applications](https://www.topcoder.com/thrive/articles/Geometry%20Concepts%20part%202:%20%20Line%20Intersection%20and%20its%20Applications)
* **SIMD Optimization:** Vectorized `immintrin.h` logic assisted by LLM-based code transformation for peak hardware utilization.
* **Framework:** Compatible with [CommonRoad](https://commonroad.in.tum.de/) XML formats.
