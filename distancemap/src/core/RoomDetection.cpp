#include "RoomDetection.hpp"

#include "Debug.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace DistanceMap {

namespace {
// ---- Union-Find (path-compressed, union-by-rank) ----------------------------

struct UnionFind {
    std::vector<int> parent, rank_;
    explicit UnionFind(int n) : parent(n), rank_(n, 0) {
        for (int i = 0; i < n; ++i) parent[i] = i;
    }
    int find(int x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    }
    void unite(int a, int b) {
        a = find(a); b = find(b);
        if (a == b) return;
        if (rank_[a] < rank_[b]) std::swap(a, b);
        parent[b] = a;
        if (rank_[a] == rank_[b]) ++rank_[a];
    }
};
} // namespace

// ---- PairHash for unordered_map<pair<int,int>, ...> -------------------------

struct IntPairHash {
    std::size_t operator()(const std::pair<int,int> &p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 16);
    }
};

// ---- makeRoomMap ------------------------------------------------------------

RoomMap makeRoomMap(const GridType::Grid &/*infoGrid*/,
                    const WallDistanceGrid &wallDist,
                    const RoomParams &params) {
    const int H = wallDist.height;
    const int W = wallDist.width;

    RoomMap result;
    result.width  = W;
    result.height = H;
    result.labels.assign(H, std::vector<int>(W, ROOM_NONE));

    if (H == 0 || W == 0) return result;

    // ---- Phase 2: Seed extraction -------------------------------------------
    // A seed is a strict local maximum in wallDist among 8 neighbours with
    // value >= MIN_SEED_DIST. Tie-break: lower (y,x) index wins.

    const int dx8[8] = {1, 1, 0,-1,-1,-1, 0, 1};
    const int dy8[8] = {0, 1, 1, 1, 0,-1,-1,-1};
    const int dx4[4] = {1,-1, 0, 0};
    const int dy4[4] = {0, 0, 1,-1};

    std::vector<std::tuple<int,int,int>> seeds; // (x, y, regionId)

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int d = wallDist.get(x, y);
            if (d < params.minSeedDist) continue;

            bool isMax = true;
            for (int k = 0; k < 8 && isMax; ++k) {
                int nx = x + dx8[k], ny = y + dy8[k];
                if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                int nd = wallDist.get(nx, ny);
                if (nd > d) { isMax = false; break; }
                // Equal value: lower (y,x) index wins, so this cell loses.
                if (nd == d && (ny < y || (ny == y && nx < x))) {
                    isMax = false; break;
                }
            }
            if (isMax) {
                int rid = static_cast<int>(seeds.size());
                seeds.emplace_back(x, y, rid);
            }
        }
    }

    int regionCount = static_cast<int>(seeds.size());
    LOG_INFO("RoomDetect: " << seeds.size() << " seeds, "
             << regionCount << " initial regions");

    if (regionCount == 0) return result;

    // ---- Phase 3: Watershed BFS (max-heap by wallDist) ----------------------

    // (wallDist, x, y, regionId)
    using Entry = std::tuple<int,int,int,int>;
    std::priority_queue<Entry> pq;

    for (auto &[sx, sy, rid] : seeds) {
        pq.emplace(wallDist.get(sx, sy), sx, sy, rid);
    }

    while (!pq.empty()) {
        auto [dist, x, y, rid] = pq.top(); pq.pop();
        if (result.labels[y][x] != ROOM_NONE) continue;
        result.labels[y][x] = rid;
        for (int k = 0; k < 4; ++k) {
            int nx = x + dx4[k], ny = y + dy4[k];
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            if (wallDist.get(nx, ny) == 0) continue;          // wall
            if (result.labels[ny][nx] != ROOM_NONE) continue; // taken

            // Gate-width check: count valid parallel crossings at this edge.
            // Each crossing is a (source, dest) pair offset perpendicularly.
            // Expansion direction (dx4[k], dy4[k]); perpendicular is (-dy4[k], dx4[k]).
            // Both the dest side AND source side must be floor to count as a crossing.
            // This avoids counting far-side room cells as part of the gate width.
            int pw = -dy4[k], ph = dx4[k];
            int gateWidth = 1; // the (x,y)->(nx,ny) crossing itself always counts
            for (int s = 1; s < params.minGateWidth && gateWidth < params.minGateWidth; ++s) {
                int cx = nx + s*pw, cy = ny + s*ph; // dest side offset
                int sx = x  + s*pw, sy = y  + s*ph; // source side offset
                if (cx < 0 || cx >= W || cy < 0 || cy >= H) break;
                if (sx < 0 || sx >= W || sy < 0 || sy >= H) break;
                if (wallDist.get(cx, cy) == 0 || wallDist.get(sx, sy) == 0) break;
                ++gateWidth;
            }
            for (int s = 1; s < params.minGateWidth && gateWidth < params.minGateWidth; ++s) {
                int cx = nx - s*pw, cy = ny - s*ph;
                int sx = x  - s*pw, sy = y  - s*ph;
                if (cx < 0 || cx >= W || cy < 0 || cy >= H) break;
                if (sx < 0 || sx >= W || sy < 0 || sy >= H) break;
                if (wallDist.get(cx, cy) == 0 || wallDist.get(sx, sy) == 0) break;
                ++gateWidth;
            }
            if (gateWidth < params.minGateWidth) continue;

            pq.emplace(wallDist.get(nx, ny), nx, ny, rid);
            LOG_DEBUG("ph3: xy; " << nx <<","<< ny << " r1: " << rid << " wallDist: "
                << static_cast<int>(wallDist.get(nx, ny)));
        }
    }

    // ---- Phase 4: Boundary classification -----------------------------------
    // gateWidth counts unique r1-side CELLS adjacent to r2 — not edges — so a
    // corner cell touching r2 in two directions still counts as one gate cell.

    struct BndInfo { int gateWidth = 0; int maxGateDist = 0; };
    std::unordered_map<std::pair<int,int>, BndInfo, IntPairHash> boundaries;

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int r1 = result.labels[y][x];
            if (r1 == ROOM_NONE) continue;
            LOG_DEBUG("ph4: xy; " << x <<","<< y << " r1: " << r1);

            int qualR2[4]; int nQual = 0;
            for (int k = 0; k < 4; ++k) {
                int nx = x + dx4[k], ny = y + dy4[k];
                if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                int r2 = result.labels[ny][nx];
                if (r2 == ROOM_NONE || r2 <= r1) continue;
                int gateDist = std::max(wallDist.get(x,y), wallDist.get(nx,ny));
                auto &bnd = boundaries[{r1, r2}];
                bnd.maxGateDist = std::max(bnd.maxGateDist, gateDist);
                LOG_DEBUG("  k=" << k << " xy: " << nx <<","<< ny << " r2: " << r2
                    << " gateDist: " << gateDist << " maxGate: " << bnd.maxGateDist);
                if (gateDist >= params.minGateDist) {
                    bool found = false;
                    for (int i = 0; i < nQual; ++i) if (qualR2[i] == r2) { found = true; break; }
                    if (!found) qualR2[nQual++] = r2;
                }
            }
            for (int i = 0; i < nQual; ++i) {
                auto &bnd = boundaries[{r1, qualR2[i]}];
                ++bnd.gateWidth;
                LOG_DEBUG("  cell adj r2=" << qualR2[i] << " => gateWidth: " << bnd.gateWidth);
            }
        }
    }

    // ---- Phase 5A: Wide-gate merge (union-find) -----------------------------

    UnionFind uf(regionCount);
    for (auto &[key, bnd] : boundaries) {
        if (bnd.gateWidth >= params.minGateWidth) {
            LOG_DEBUG("UNITE: r1,r2 (" << key.first << ", " << key.second << ")"
                << " maxGate: " << bnd.maxGateDist << " gateWidth: " << bnd.gateWidth);
            uf.unite(key.first, key.second);
        }
    }

    // Remap labels to roots
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            if (result.labels[y][x] != ROOM_NONE)
                result.labels[y][x] = uf.find(result.labels[y][x]);

    // Compact root ids to 0..N-1
    std::unordered_map<int,int> compact;
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int lbl = result.labels[y][x];
            if (lbl == ROOM_NONE) continue;
            if (compact.find(lbl) == compact.end()) {
                int newId = static_cast<int>(compact.size());
                compact[lbl] = newId;
            }
        }
    }
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            if (result.labels[y][x] != ROOM_NONE)
                result.labels[y][x] = compact[result.labels[y][x]];

    int compactCount = static_cast<int>(compact.size());

    // ---- Phase 5B: Under-area discard ---------------------------------------

    std::vector<int> area(compactCount, 0);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            if (result.labels[y][x] != ROOM_NONE)
                ++area[result.labels[y][x]];

    std::vector<bool> discard(compactCount, false);
    for (int i = 0; i < compactCount; ++i)
        if (area[i] < params.minArea) discard[i] = true;

    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            if (result.labels[y][x] != ROOM_NONE && discard[result.labels[y][x]])
                result.labels[y][x] = ROOM_NONE;

    // Re-compact after discard to get consecutive ids
    compact.clear();
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int lbl = result.labels[y][x];
            if (lbl == ROOM_NONE) continue;
            if (compact.find(lbl) == compact.end()) {
                int newId = static_cast<int>(compact.size());
                compact[lbl] = newId;
            }
        }
    }
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            if (result.labels[y][x] != ROOM_NONE)
                result.labels[y][x] = compact[result.labels[y][x]];

    int finalCount = static_cast<int>(compact.size());
    LOG_INFO("RoomDetect: " << finalCount << " rooms after merge/discard");

    if (finalCount == 0) return result;

    // ---- Phase 6: Statistics ------------------------------------------------

    result.rooms.resize(finalCount);
    std::vector<int> minX(finalCount, INT_MAX), minY(finalCount, INT_MAX);
    std::vector<int> maxX(finalCount, 0),       maxY(finalCount, 0);

    for (int i = 0; i < finalCount; ++i) {
        result.rooms[i].id          = i;
        result.rooms[i].area        = 0;
        result.rooms[i].maxWallDist = 0;
        result.rooms[i].center      = {0, 0};
    }

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int rid = result.labels[y][x];
            if (rid == ROOM_NONE) continue;
            RoomRegion &r = result.rooms[rid];
            r.area++;
            int d = wallDist.get(x, y);
            if (d > r.maxWallDist) {
                r.maxWallDist = d;
                r.center = {x, y};
            }
            minX[rid] = std::min(minX[rid], x);
            minY[rid] = std::min(minY[rid], y);
            maxX[rid] = std::max(maxX[rid], x);
            maxY[rid] = std::max(maxY[rid], y);
        }
    }

    for (int i = 0; i < finalCount; ++i) {
        result.rooms[i].approxWidth  = maxX[i] - minX[i] + 1;
        result.rooms[i].approxHeight = maxY[i] - minY[i] + 1;
    }

    // Neighbour list: rooms reachable across any boundary (gateWidth >= 1)
    // Re-scan boundaries with final compact ids
    std::unordered_map<std::pair<int,int>, bool, IntPairHash> neighborPairs;
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int r1 = result.labels[y][x];
            if (r1 == ROOM_NONE) continue;
            for (int k = 0; k < 4; ++k) {
                int nx = x + dx4[k], ny = y + dy4[k];
                if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                int r2 = result.labels[ny][nx];
                if (r2 == ROOM_NONE || r2 == r1) continue;
                neighborPairs[{std::min(r1,r2), std::max(r1,r2)}] = true;
            }
        }
    }
    for (auto &[key, _] : neighborPairs) {
        result.rooms[key.first].neighborRoomIds.push_back(key.second);
        result.rooms[key.second].neighborRoomIds.push_back(key.first);
    }

    for (int i = 0; i < finalCount; ++i) {
        const RoomRegion &r = result.rooms[i];
        LOG_INFO("RoomDetect: Room " << r.id
            << " area=" << r.area
            << " center=(" << r.center.first << "," << r.center.second << ")"
            << " maxDist=" << r.maxWallDist
            << " size=" << r.approxWidth << "x" << r.approxHeight
            << " neighbours=" << r.neighborRoomIds.size());

    }
    LOG_DEBUG("## Rooms Map");
    for (int y = 0; y < result.height; ++y) {
      for (int x = 0; x < result.width; ++x) {
        if (wallDist.get(x,y) == 0) {
          LOG_DEBUG_CONT("#");
        } else {
          int lbl = result.labels[y][x];
          if (lbl == ROOM_NONE) {
            LOG_DEBUG_CONT(".");
          } else if (lbl < 10) {
            LOG_DEBUG_CONT(static_cast<char>('0' + lbl));
          } else if (lbl < 36) {
            LOG_DEBUG_CONT(static_cast<char>('a' + (lbl - 10)));
          } else {
            LOG_DEBUG_CONT(" ");
          }
        }
      }
      LOG_DEBUG("");
    }
    return result;
}

// ---- detectCorridors --------------------------------------------------------
// See SPECIFICATION: specs/systems/corridor_detection.md

void detectCorridors(RoomMap &roomMap,
                     const std::vector<GridType::Point> &baseNodes,
                     const std::vector<GridType::Edge> &baseEdges,
                     const WallDistanceGrid &wallDist) {
    roomMap.corridors.clear();
    roomMap.corridorLabels.assign(roomMap.height,
                                  std::vector<int>(roomMap.width, ROOM_NONE));
    const int N = static_cast<int>(baseNodes.size());
    if (N == 0 || roomMap.rooms.empty()) return;

    auto labelAt = [&](const GridType::Point &p) -> int {
        int x = p.first, y = p.second;
        if (y < 0 || y >= roomMap.height || x < 0 || x >= roomMap.width)
            return ROOM_NONE;
        return roomMap.labels[y][x];
    };

    // Classify each edge by the room labels at its two path-cell ends.
    // Room attachment is keyed on path-end CELL labels (not baseNode cells):
    // rooms are open blobs and the skeleton often dead-ends *inside* a room, so
    // a dead-end edge can legitimately link two rooms across the ROOM_NONE gap.
    //   room <-> room (different) : direct corridor (whole edge, dead or not)
    //   room <-> ROOM_NONE        : port (only non-dead: the ROOM_NONE end is a
    //                               chainable baseNode)
    //   ROOM_NONE <-> ROOM_NONE   : corridor-corridor adjacency (non-dead)
    std::vector<std::vector<std::pair<int,int>>> adj(N); // (neighborNode, edgeIdx)
    struct Port { int corridorNode; int room; int edgeIdx; int roomNode; };
    std::vector<Port> ports;
    std::vector<int> directEdges; // edge indices linking two different rooms

    for (int e = 0; e < static_cast<int>(baseEdges.size()); ++e) {
        const auto &E = baseEdges[e];
        if (E.path.empty()) continue;
        int rF = labelAt(E.path.front());
        int rB = labelAt(E.path.back());
        bool dead = E.toDeadEnd;

        if (rF != ROOM_NONE && rB != ROOM_NONE) {
            if (rF != rB) directEdges.push_back(e); // else intra-room: ignore
        } else if (rF != ROOM_NONE && rB == ROOM_NONE) {
            if (!dead && E.to >= 0 && E.to < N)
                ports.push_back({E.to, rF, e, E.from}); // mouth into rF at front
        } else if (rF == ROOM_NONE && rB != ROOM_NONE) {
            if (!dead && E.from >= 0 && E.from < N)
                ports.push_back({E.from, rB, e, E.to});
        } else { // both ROOM_NONE
            if (!dead && E.from >= 0 && E.from < N && E.to >= 0 && E.to < N) {
                adj[E.from].push_back({E.to, e});
                adj[E.to].push_back({E.from, e});
            }
        }
    }

    // Shortest corridor-node path (by node count) over adj, deterministic.
    auto bfsPath = [&](int src, int dst) -> std::vector<int> {
        if (src == dst) return {src};
        std::vector<int> prev(N, -1);
        std::vector<char> seen(N, 0);
        std::queue<int> q;
        q.push(src); seen[src] = 1;
        while (!q.empty()) {
            int c = q.front(); q.pop();
            if (c == dst) break;
            for (auto &pr : adj[c])
                if (!seen[pr.first]) { seen[pr.first] = 1; prev[pr.first] = c; q.push(pr.first); }
        }
        if (!seen[dst]) return {};
        std::vector<int> seq;
        for (int c = dst; c != -1; c = prev[c]) seq.push_back(c);
        std::reverse(seq.begin(), seq.end());
        return seq;
    };

    auto edgeBetween = [&](int a, int b) -> int {
        for (auto &pr : adj[a]) if (pr.first == b) return pr.second;
        return -1;
    };

    // Build an ordered centerline cell list from a baseNode sequence and the
    // edges connecting consecutive nodes. Consecutive duplicate cells dropped.
    auto buildCenterline = [&](const std::vector<int> &seq,
                               const std::vector<int> &edgeSeq) -> GridType::Path {
        GridType::Path cl;
        auto pushCell = [&](const GridType::Point &p) {
            if (cl.empty() || cl.back() != p) cl.push_back(p);
        };
        if (seq.empty()) return cl;
        pushCell(baseNodes[seq[0]]);
        for (size_t k = 0; k + 1 < seq.size(); ++k) {
            int e = edgeSeq[k];
            if (e >= 0) {
                const auto &E = baseEdges[e];
                if (E.from == seq[k])
                    for (auto &c : E.path) pushCell(c);
                else
                    for (auto it = E.path.rbegin(); it != E.path.rend(); ++it) pushCell(*it);
            }
            pushCell(baseNodes[seq[k + 1]]);
        }
        return cl;
    };

    // Fill length/wiggle/thickness for a corridor from its centerline.
    auto computeStats = [&](Corridor &c) {
        const GridType::Path &cl = c.centerline;
        c.length = static_cast<int>(cl.size());
        if (cl.empty()) return;
        c.start = cl.front();
        c.end   = cl.back();

        // Wiggle: accumulated absolute turn angle / length.
        float totalTurn = 0.0f;
        bool haveDir = false;
        float pdx = 0.0f, pdy = 0.0f;
        for (size_t i = 1; i < cl.size(); ++i) {
            float dx = static_cast<float>(cl[i].first  - cl[i-1].first);
            float dy = static_cast<float>(cl[i].second - cl[i-1].second);
            if (dx == 0.0f && dy == 0.0f) continue;
            float len = std::sqrt(dx*dx + dy*dy);
            dx /= len; dy /= len;
            if (haveDir) {
                float dot = pdx*dx + pdy*dy;
                float cross = pdx*dy - pdy*dx;
                totalTurn += std::fabs(std::atan2(cross, dot));
            }
            pdx = dx; pdy = dy; haveDir = true;
        }
        c.wiggle = c.length > 0 ? totalTurn / static_cast<float>(c.length) : 0.0f;

        // Thickness: diameter = 2*wallDist+1 per cell, bucketed.
        int tight = 0, normal = 0, wide = 0;
        int minD = INT_MAX, maxD = 0;
        long sum = 0;
        for (auto &p : cl) {
            int diam = 2 * static_cast<int>(wallDist.get(p.first, p.second)) + 1;
            if (diam <= CORRIDOR_DIAM_TIGHT_MAX) ++tight;
            else if (diam <= CORRIDOR_DIAM_NORMAL_MAX) ++normal;
            else ++wide;
            minD = std::min(minD, diam);
            maxD = std::max(maxD, diam);
            sum += diam;
        }
        int tot = c.length;
        c.thickness.fracTight  = static_cast<float>(tight)  / tot;
        c.thickness.fracNormal = static_cast<float>(normal) / tot;
        c.thickness.fracWide   = static_cast<float>(wide)   / tot;
        c.thickness.avgDiameter = static_cast<int>(sum / tot);
        c.thickness.minDiameter = minD;
        c.thickness.maxDiameter = maxD;
    };

    // Index range [first, last] of the ROOM_NONE cells in a path, or {-1,-1} if
    // none (rooms physically touch). This is the corridor span between rooms.
    auto corridorSpan = [&](const GridType::Path &p) -> std::pair<int,int> {
        int first = -1, last = -1;
        for (int i = 0; i < static_cast<int>(p.size()); ++i)
            if (labelAt(p[i]) == ROOM_NONE) { if (first < 0) first = i; last = i; }
        return {first, last};
    };

    // Trim leading/trailing room-labeled cells so the centerline (and start/end)
    // covers just the ROOM_NONE span. Paths with no ROOM_NONE cells are kept.
    auto trimToCorridorSpan = [&](GridType::Path &cl) {
        auto [first, last] = corridorSpan(cl);
        if (first < 0) return;
        cl = GridType::Path(cl.begin() + first, cl.begin() + last + 1);
    };

    // Track which corridor-node path each corridor traversed, for crossings.
    std::vector<std::vector<int>> corridorNodePaths;

    // ---- Enumerate room-pair corridors over port pairs ----------------------
    const int P = static_cast<int>(ports.size());
    for (int i = 0; i < P; ++i) {
        for (int j = i + 1; j < P; ++j) {
            if (ports[i].room == ports[j].room) continue; // same room: skip
            std::vector<int> corPath =
                bfsPath(ports[i].corridorNode, ports[j].corridorNode);
            if (corPath.empty()) continue; // disconnected networks

            std::vector<int> seq;
            seq.push_back(ports[i].roomNode);
            for (int n : corPath) seq.push_back(n);
            seq.push_back(ports[j].roomNode);

            std::vector<int> edgeSeq;
            edgeSeq.push_back(ports[i].edgeIdx);
            for (size_t k = 0; k + 1 < corPath.size(); ++k)
                edgeSeq.push_back(edgeBetween(corPath[k], corPath[k+1]));
            edgeSeq.push_back(ports[j].edgeIdx);

            Corridor c;
            c.id = static_cast<int>(roomMap.corridors.size());
            c.roomA = ports[i].room;
            c.roomB = ports[j].room;
            c.centerline = buildCenterline(seq, edgeSeq);
            trimToCorridorSpan(c.centerline);
            computeStats(c);
            roomMap.corridors.push_back(std::move(c));
            corridorNodePaths.push_back(std::move(corPath));
        }
    }

    // ---- Direct room-to-room skeleton edges (incl. dead-end stubs) ----------
    // The edge path runs room -> ROOM_NONE gap -> room. Trim to the gap; the
    // rooms are the labels of the cells flanking the trimmed span.
    for (int e : directEdges) {
        const auto &E = baseEdges[e];
        const int n = static_cast<int>(E.path.size());
        auto [first, last] = corridorSpan(E.path);

        Corridor c;
        c.id = static_cast<int>(roomMap.corridors.size());
        if (first < 0) { // no gap: rooms touch, keep whole edge
            c.centerline = E.path;
            c.roomA = labelAt(E.path.front());
            c.roomB = labelAt(E.path.back());
        } else {
            c.centerline = GridType::Path(E.path.begin() + first,
                                          E.path.begin() + last + 1);
            // Rooms are the cells flanking the trimmed span.
            c.roomA = first > 0      ? labelAt(E.path[first - 1]) : labelAt(E.path.front());
            c.roomB = last + 1 < n   ? labelAt(E.path[last + 1])  : labelAt(E.path.back());
        }
        computeStats(c);
        roomMap.corridors.push_back(std::move(c));
        corridorNodePaths.push_back({}); // no corridor nodes -> no crossings
    }

    // ---- Crossings: corridors sharing a junction baseNode -------------------
    std::unordered_map<int, std::vector<int>> nodeToCorr; // junctionNode -> corridorIds
    for (size_t ci = 0; ci < corridorNodePaths.size(); ++ci) {
        for (int node : corridorNodePaths[ci]) {
            if (static_cast<int>(adj[node].size()) < 3) continue; // not a junction
            auto &ids = nodeToCorr[node];
            if (std::find(ids.begin(), ids.end(), static_cast<int>(ci)) == ids.end())
                ids.push_back(static_cast<int>(ci));
        }
    }
    for (auto &kv : nodeToCorr) {
        const std::vector<int> &ids = kv.second;
        if (ids.size() < 2) continue;
        for (int id : ids)
            for (int other : ids)
                if (other != id)
                    roomMap.corridors[id].crossings.push_back(
                        {other, baseNodes[kv.first]});
    }

    // Stamp centerline cells with their corridor id (first writer wins at any
    // junction cell shared by multiple corridors).
    for (const auto &c : roomMap.corridors) {
        for (const auto &p : c.centerline) {
            int x = p.first, y = p.second;
            if (x < 0 || x >= roomMap.width || y < 0 || y >= roomMap.height) continue;
            if (roomMap.corridorLabels[y][x] == ROOM_NONE)
                roomMap.corridorLabels[y][x] = c.id;
        }
    }

    LOG_INFO("CorridorDetect: " << roomMap.corridors.size() << " corridors");
    for (const auto &c : roomMap.corridors) {
        LOG_INFO("CorridorDetect: Corr " << c.id
            << " rooms=" << c.roomA << "<->" << c.roomB
            << " start=(" << c.start.first << "," << c.start.second << ")"
            << " end=(" << c.end.first << "," << c.end.second << ")"
            << " len=" << c.length
            << " wiggle=" << c.wiggle
            << " diam(avg/min/max)=" << c.thickness.avgDiameter
            << "/" << c.thickness.minDiameter
            << "/" << c.thickness.maxDiameter
            << " crossings=" << c.crossings.size());
    }
}

} // namespace DistanceMap
