#include "RoomDetection.hpp"

#include "Debug.h"

#include <climits>
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
                    const WallDistanceGrid &wallDist) {
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
            if (d < MIN_SEED_DIST) continue;

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
            pq.emplace(wallDist.get(nx, ny), nx, ny, rid);
        }
    }

    // ---- Phase 4: Boundary classification -----------------------------------

    struct BndInfo { int gateWidth = 0; int maxGateDist = 0; };
    std::unordered_map<std::pair<int,int>, BndInfo, IntPairHash> boundaries;

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int r1 = result.labels[y][x];
            if (r1 == ROOM_NONE) continue;
            for (int k = 0; k < 4; ++k) {
                int nx = x + dx4[k], ny = y + dy4[k];
                if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                int r2 = result.labels[ny][nx];
                if (r2 == ROOM_NONE || r2 == r1) continue;
                auto key = std::make_pair(std::min(r1,r2), std::max(r1,r2));
                int gateDist = std::max(wallDist.get(x,y), wallDist.get(nx,ny));
                auto &bnd = boundaries[key];
                bnd.maxGateDist = std::max(bnd.maxGateDist, gateDist);
                if (gateDist >= MIN_GATE_DIST)
                    bnd.gateWidth++;
            }
        }
    }

    // ---- Phase 5A: Wide-gate merge (union-find) -----------------------------

    UnionFind uf(regionCount);
    for (auto &[key, bnd] : boundaries) {
        if (bnd.gateWidth >= MIN_GATE_WIDTH)
            uf.unite(key.first, key.second);
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
        if (area[i] < MIN_AREA) discard[i] = true;

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

    return result;
}

} // namespace DistanceMap
