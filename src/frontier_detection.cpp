#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>
#include <unordered_map>
#include <mutex>
#include <queue>
#include <cmath>

#include <dynamic_reconfigure/server.h>
#include <firefly_control/FrontierDetectionConfig.h>

struct Vec3
{
    double x, y, z;
};

enum VoxelState
{
    UNKNOWN = 0,
    FREE = 1,
    OCCUPIED = 2
};

// -------------------- VISITED GRID (SPATIAL HASH) --------------------
// Bu sınıf visited noktalarını hücrelere koyar; isNearVisited sorgusu sadece
// ilgili hücre + 26 komşuyu kontrol eder -> O(1) amortize performans.
struct Key
{
    int ix, iy, iz;
    bool operator==(const Key &o) const { return ix == o.ix && iy == o.iy && iz == o.iz; }
};
struct KeyHasher
{
    size_t operator()(const Key &k) const noexcept
    {
        // basit karıştırma
        uint64_t a = (uint64_t)(k.ix) + 0x9e3779b97f4a7c15ULL;
        uint64_t b = (uint64_t)(k.iy) + 0x9e3779b97f4a7c15ULL;
        uint64_t c = (uint64_t)(k.iz) + 0x9e3779b97f4a7c15ULL;
        uint64_t res = a;
        res ^= b + 0x9e3779b97f4a7c15ULL + (res << 6) + (res >> 2);
        res ^= c + 0x9e3779b97f4a7c15ULL + (res << 6) + (res >> 2);
        return (size_t)res;
    }
};

class VisitedGrid
{
public:
    VisitedGrid() : cell_size_(1.0), radius_(1.0) {}

    // Build the grid from visited positions (call when visited_positions_ updated)
    void build(double radius, const std::vector<Vec3> &visited_positions)
    {
        radius_ = radius;
        // make cell size equal to radius (or choose slightly smaller/larger if desired)
        cell_size_ = std::max(0.0001, radius_);
        grid_.clear();

        for (const auto &p : visited_positions)
        {
            Key k = toKey(p);
            grid_[k].push_back(p);
        }
    }

    bool isNearVisited(const Vec3 &v) const
    {
        if (grid_.empty())
            return false;
        Key k = toKey(v);
        // check 3x3x3 neighbors only
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    Key nk{k.ix + dx, k.iy + dy, k.iz + dz};
                    auto it = grid_.find(nk);
                    if (it == grid_.end())
                        continue;
                    for (const auto &p : it->second)
                    {
                        double dist2 = (v.x - p.x) * (v.x - p.x) + (v.y - p.y) * (v.y - p.y) + (v.z - p.z) * (v.z - p.z);
                        if (dist2 < radius_ * radius_)
                            return true;
                    }
                }
        return false;
    }

private:
    Key toKey(const Vec3 &v) const
    {
        return Key{
            static_cast<int>(std::floor(v.x / cell_size_)),
            static_cast<int>(std::floor(v.y / cell_size_)),
            static_cast<int>(std::floor(v.z / cell_size_))};
    }

    double cell_size_;
    double radius_;
    std::unordered_map<Key, std::vector<Vec3>, KeyHasher> grid_;
};
// -------------------- /VISITED GRID --------------------

// Union-Find
class DisjointSet
{
public:
    void makeSet(int x)
    {
        parent[x] = x;
        rank[x] = 0;
    }
    int find(int x)
    {
        if (parent[x] != x)
            parent[x] = find(parent[x]);
        return parent[x];
    }
    void unite(int x, int y)
    {
        int px = find(x), py = find(y);
        if (px == py)
            return;
        if (rank[px] < rank[py])
            parent[px] = py;
        else if (rank[px] > rank[py])
            parent[py] = px;
        else
        {
            parent[py] = px;
            rank[px]++;
        }
    }

private:
    std::unordered_map<int, int> parent, rank;
};

// A* Node
struct Node
{
    int x, y, z;
    double g, h;
    Node *parent;
    Node(int x_, int y_, int z_, double g_, double h_, Node *parent_) : x(x_), y(y_), z(z_), g(g_), h(h_), parent(parent_) {}
    double f() const { return g + h; }
};

class OctoVoxelCCRViz
{
public:
    OctoVoxelCCRViz(double voxel_size, const std::string &frame, int min_voxels,
                    double xmin_allowed, double xmax_allowed,
                    double ymin_allowed, double ymax_allowed,
                    double visited_radius)
        : voxel_size_(voxel_size), tree_(nullptr), frame_(frame), min_voxels_per_group_(min_voxels),
          xmin_allowed_(xmin_allowed), xmax_allowed_(xmax_allowed),
          ymin_allowed_(ymin_allowed), ymax_allowed_(ymax_allowed), safe_distance_(0.1),
          floor_height_(2.5), half_band_(0.5), visited_radius_(visited_radius)
    {
        odom_sub_ = nh_.subscribe("/firefly/odometry_sensor1/odometry", 1, &OctoVoxelCCRViz::odomCallback, this);
        traj_sub_ = nh_.subscribe("/firefly/traj", 1, &OctoVoxelCCRViz::trajCallback, this);

        cb_ = boost::bind(&OctoVoxelCCRViz::reconfigCallback, this, _1, _2);
        dr_srv_.setCallback(cb_);
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        octomap::AbstractOcTree *absTree = octomap_msgs::fullMsgToMap(*msg);
        if (!absTree)
            return;

        if (tree_)
            delete tree_;
        tree_ = dynamic_cast<octomap::OcTree *>(absTree);

        buildVoxelGrid();
        findFrontiersAndCC();
        computeTargets();
    }

private:
    double voxel_size_;
    double xmin_allowed_, xmax_allowed_, ymin_allowed_, ymax_allowed_;
    double visited_radius_;
    double z_tolerance_ = 1.0;

    octomap::OcTree *tree_;
    std::string frame_;
    int min_voxels_per_group_;
    std::vector<std::vector<std::vector<VoxelState>>> grid_;
    int nx_, ny_, nz_;
    double xmin_, ymin_, zmin_;
    std::mutex mutex_;
    std::vector<Vec3> frontierVoxels_;
    std::unordered_map<int, std::vector<Vec3>> components_;

    std::vector<Vec3> visited_positions_;
    VisitedGrid visited_grid_; // <-- yeni member

    dynamic_reconfigure::Server<firefly_control::FrontierDetectionConfig> dr_srv_;
    dynamic_reconfigure::Server<firefly_control::FrontierDetectionConfig>::CallbackType cb_;

    double safe_distance_;
    Vec3 robot_pos_;

    ros::NodeHandle nh_;
    ros::Publisher target_pub_ = nh_.advertise<geometry_msgs::PointStamped>("cc_targets", 1);
    ros::Publisher pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cc_targets_cloud", 1);
    ros::Subscriber odom_sub_;
    ros::Subscriber traj_sub_;

    double floor_height_;
    double half_band_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        robot_pos_.x = msg->pose.pose.position.x;
        robot_pos_.y = msg->pose.pose.position.y;
        robot_pos_.z = msg->pose.pose.position.z;
    }

    void trajCallback(const visualization_msgs::Marker::ConstPtr &traj_msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        visited_positions_.clear();
        for (const auto &p : traj_msg->points)
        {
            visited_positions_.push_back({p.x, p.y, p.z});
        }
        // visited grid'i güncelle (hızlı sorgular için)
        visited_grid_.build(visited_radius_, visited_positions_);
    }

    void reconfigCallback(firefly_control::FrontierDetectionConfig &config, uint32_t level)
    {
        voxel_size_ = config.voxel_size;
        min_voxels_per_group_ = config.min_voxels_per_group;
        frame_ = config.frame_id;
        safe_distance_ = config.safe_distance;
        frontier_min_unknown_neighbors_ = config.frontier_min_unknown_neighbors;
        visited_radius_ = config.visited_radius; // dynamic param
        z_tolerance_ = config.z_tolerance;

        ROS_INFO("Reconfigure: frontier_min_unknown_neighbors=%d, voxel_size=%.2f, min_voxels=%d, safe_dist=%.2f, visited_radius=%.2f, frame=%s",
                 frontier_min_unknown_neighbors_, voxel_size_, min_voxels_per_group_, safe_distance_, visited_radius_, frame_.c_str());
        // rebuild visited_grid if we already have visited positions
        visited_grid_.build(visited_radius_, visited_positions_);
    }

    void buildVoxelGrid()
    {
        if (!tree_)
            return;
        tree_->getMetricMin(xmin_, ymin_, zmin_);
        double xmax, ymax, zmax;
        tree_->getMetricMax(xmax, ymax, zmax);
        nx_ = static_cast<int>((xmax - xmin_) / voxel_size_) + 1;
        ny_ = static_cast<int>((ymax - ymin_) / voxel_size_) + 1;
        nz_ = static_cast<int>((zmax - zmin_) / voxel_size_) + 1;

        grid_.assign(nx_, std::vector<std::vector<VoxelState>>(ny_, std::vector<VoxelState>(nz_, UNKNOWN)));

        for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it)
        {
            int ix = static_cast<int>((it.getX() - xmin_) / voxel_size_);
            int iy = static_cast<int>((it.getY() - ymin_) / voxel_size_);
            int iz = static_cast<int>((it.getZ() - zmin_) / voxel_size_);
            if (ix >= 0 && iy >= 0 && iz >= 0 && ix < nx_ && iy < ny_ && iz < nz_)
            {
                grid_[ix][iy][iz] = tree_->isNodeOccupied(*it) ? OCCUPIED : FREE;
            }
        }
    }

    bool isVoxelInActiveBand(double z, int floor_idx)
    {
        double band_center = floor_idx * floor_height_ + floor_height_ / 2.0;
        return std::abs(z - band_center) <= half_band_;
    }

    bool isFrontier(int ix, int iy, int iz)
    {
        if (grid_[ix][iy][iz] != FREE)
            return false;

        double z = zmin_ + iz * voxel_size_;
        int floor_idx = static_cast<int>(z / floor_height_);
        if (!isVoxelInActiveBand(z, floor_idx))
            return false;

        int unknown_count = 0;

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;
                    int nx = ix + dx, ny = iy + dy, nz = iz + dz;
                    if (nx < 0 || ny < 0 || nz < 0 || nx >= nx_ || ny >= ny_ || nz >= nz_)
                        continue;

                    if (grid_[nx][ny][nz] == OCCUPIED)
                        return false;

                    if (grid_[nx][ny][nz] == UNKNOWN)
                        unknown_count++;
                }

        return unknown_count >= frontier_min_unknown_neighbors_;
    }

    Vec3 computeCentroid(const std::vector<Vec3> &voxels, int floor_idx)
    {
        Vec3 c{0, 0, 0};
        int count = 0;
        double band_center = floor_idx * floor_height_ + floor_height_ / 2.0;
        for (const auto &v : voxels)
        {
            if (std::abs(v.z - band_center) > half_band_)
                continue;
            c.x += v.x;
            c.y += v.y;
            c.z += v.z;
            count++;
        }
        if (count == 0)
            return {0, 0, 0};
        c.x /= count;
        c.y /= count;
        c.z = band_center;
        return c;
    }

    void findFrontiersAndCC()
    {
        frontierVoxels_.clear();
        components_.clear();
        DisjointSet uf;

        int max_floor_idx = static_cast<int>((zmin_ + nz_ * voxel_size_) / floor_height_);

        // Eğer visited_grid boşsa, yine de build edelim (visited_positions_ güncellenmiş olabilir)
        visited_grid_.build(visited_radius_, visited_positions_);

        for (int floor_idx = 0; floor_idx <= max_floor_idx; floor_idx++)
        {
            for (int ix = 0; ix < nx_; ix++)
                for (int iy = 0; iy < ny_; iy++)
                    for (int iz = 0; iz < nz_; iz++)
                    {
                        double z = zmin_ + iz * voxel_size_;
                        if (!isVoxelInActiveBand(z, floor_idx))
                            continue;
                        if (isFrontier(ix, iy, iz))
                        {
                            Vec3 v{xmin_ + ix * voxel_size_, ymin_ + iy * voxel_size_, z};
                            // Burada artık spatial-hash ile hızlı kontrol yapıyoruz
                            if (visited_grid_.isNearVisited(v))
                                continue; // geçtiğimiz yere çok yakın, atla (çok hızlı)
                            frontierVoxels_.push_back(v);

                            int idx = ix + iy * nx_ + iz * nx_ * ny_;
                            uf.makeSet(idx);

                            for (int dx = -1; dx <= 1; dx++)
                                for (int dy = -1; dy <= 1; dy++)
                                    for (int dz = -1; dz <= 1; dz++)
                                    {
                                        if (dx == 0 && dy == 0 && dz == 0)
                                            continue;
                                        int nx = ix + dx, ny = iy + dy, nz = iz + dz;
                                        if (nx < 0 || ny < 0 || nz < 0 || nx >= nx_ || ny >= ny_ || nz >= nz_)
                                            continue;
                                        double nz_val = zmin_ + nz * voxel_size_;
                                        if (!isVoxelInActiveBand(nz_val, floor_idx))
                                            continue;
                                        if (isFrontier(nx, ny, nz))
                                        {
                                            int nidx = nx + ny * nx_ + nz * nx_ * ny_;
                                            uf.makeSet(nidx);
                                            uf.unite(idx, nidx);
                                        }
                                    }
                        }
                    }
        }

        // frontierVoxels_'tan bileşenleri oluştur
        for (auto &v : frontierVoxels_)
        {
            int ix = static_cast<int>((v.x - xmin_) / voxel_size_);
            int iy = static_cast<int>((v.y - ymin_) / voxel_size_);
            int iz = static_cast<int>((v.z - zmin_) / voxel_size_);
            int idx = ix + iy * nx_ + iz * nx_ * ny_;
            int root = uf.find(idx);
            components_[root].push_back(v);
        }
    }

    // BFS ile en yakın free voxel bulma
    bool findNearestFreeVoxel(int sx, int sy, int sz, Vec3 &free_voxel)
    {
        std::queue<std::tuple<int, int, int>> q;
        std::set<std::tuple<int, int, int>> visited;

        q.push({sx, sy, sz});
        visited.insert({sx, sy, sz});

        std::vector<std::array<int, 3>> dirs = {
            {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

        while (!q.empty())
        {
            auto [x, y, z] = q.front();
            q.pop();

            if (x < 0 || y < 0 || z < 0 || x >= nx_ || y >= ny_ || z >= nz_)
                continue;

            // Hücre FREE mi?
            if (grid_[x][y][z] == FREE)
            {
                bool near_occupied = false;
                for (auto &d : dirs)
                {
                    int nx = x + d[0], ny = y + d[1], nz = z + d[2];
                    if (nx < 0 || ny < 0 || nz < 0 || nx >= nx_ || ny >= ny_ || nz >= nz_)
                        continue;
                    if (grid_[nx][ny][nz] == OCCUPIED)
                    {
                        near_occupied = true;
                        break;
                    }
                }

                if (!near_occupied) // sadece güvenli hücreyi kabul et
                {
                    free_voxel.x = xmin_ + x * voxel_size_;
                    free_voxel.y = ymin_ + y * voxel_size_;
                    free_voxel.z = zmin_ + z * voxel_size_;
                    return true;
                }
                // Eğer occupied'a komşuysa bu voxel'i atla
            }

            for (auto &d : dirs)
            {
                int nx = x + d[0], ny = y + d[1], nz = z + d[2];
                auto key = std::make_tuple(nx, ny, nz);
                if (visited.count(key) == 0)
                {
                    q.push(key);
                    visited.insert(key);
                }
            }
        }
        return false; // güvenli voxel bulunamadı
    }

    void computeTargets()
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        int color_step = 255 / std::max(1, (int)components_.size());
        int color_idx = 0;

        std::vector<Vec3> valid_targets_same_floor;
        std::vector<Vec3> valid_targets_other;

        // Frontierleri gez
        for (auto &c : components_)
        {
            if (c.second.size() < min_voxels_per_group_)
                continue;

            Vec3 centroid = computeCentroid(c.second, static_cast<int>(c.second[0].z / floor_height_));
            if (centroid.x == 0 && centroid.y == 0 && centroid.z == 0)
                continue;

            if (visited_grid_.isNearVisited(centroid))
                continue;

            int ix = static_cast<int>((centroid.x - xmin_) / voxel_size_);
            int iy = static_cast<int>((centroid.y - ymin_) / voxel_size_);
            int iz = static_cast<int>((centroid.z - zmin_) / voxel_size_);
            if (ix < 0 || iy < 0 || iz < 0 || ix >= nx_ || iy >= ny_ || iz >= nz_)
                continue;
            if (grid_[ix][iy][iz] != FREE)
                continue;

            double dist_to_robot = std::sqrt(
                std::pow(centroid.x - robot_pos_.x, 2) +
                std::pow(centroid.y - robot_pos_.y, 2) +
                std::pow(centroid.z - robot_pos_.z, 2));
            if (dist_to_robot < 1.5)
                continue;

            if (centroid.x < xmin_allowed_ || centroid.x > xmax_allowed_ ||
                centroid.y < ymin_allowed_ || centroid.y > ymax_allowed_)
                continue;

            // z farkına göre ayır
            double dz = std::abs(centroid.z - robot_pos_.z);
            if (dz <= z_tolerance_)
                valid_targets_same_floor.push_back(centroid);
            else
                valid_targets_other.push_back(centroid);
        }

        // Eğer kendi katında hedef varsa sadece onu kullan
        std::vector<Vec3> valid_targets;
        if (!valid_targets_same_floor.empty())
            valid_targets = valid_targets_same_floor;
        else
            valid_targets = valid_targets_other;

        // Mesafeye göre sırala
        std::sort(valid_targets.begin(), valid_targets.end(), [&](const Vec3 &a, const Vec3 &b)
                  {
                  double da = std::sqrt(std::pow(a.x - robot_pos_.x, 2) +
                                        std::pow(a.y - robot_pos_.y, 2) +
                                        std::pow(a.z - robot_pos_.z, 2));
                  double db = std::sqrt(std::pow(b.x - robot_pos_.x, 2) +
                                        std::pow(b.y - robot_pos_.y, 2) +
                                        std::pow(b.z - robot_pos_.z, 2));
                  return da < db; });

        Vec3 best_target;
        std::vector<Vec3> best_path;
        double best_cost = std::numeric_limits<double>::infinity();

        for (auto &t : valid_targets)
        {
            // --- Hedef düzeltmesi ---
            Vec3 corrected_goal = t;
            int gx = static_cast<int>((t.x - xmin_) / voxel_size_);
            int gy = static_cast<int>((t.y - ymin_) / voxel_size_);
            int gz = static_cast<int>((t.z - zmin_) / voxel_size_);

            if (gx < 0 || gy < 0 || gz < 0 || gx >= nx_ || gy >= ny_ || gz >= nz_)
                continue;

            if (grid_[gx][gy][gz] != FREE)
            {
                if (!findNearestFreeVoxel(gx, gy, gz, corrected_goal))
                {
                    continue; // bu hedefi atla
                }
            }

            // --- Path planlama ---
            std::vector<Vec3> path;
            if (aStarPath(robot_pos_, corrected_goal, path))
            {
                double cost = 0.0;
                for (size_t i = 1; i < path.size(); i++)
                    cost += std::sqrt(std::pow(path[i].x - path[i - 1].x, 2) +
                                      std::pow(path[i].y - path[i - 1].y, 2) +
                                      std::pow(path[i].z - path[i - 1].z, 2));
                if (cost < best_cost)
                {
                    best_cost = cost;
                    best_target = corrected_goal;
                    best_path = path;
                }
            }
        }

        // Nokta bulutu publish
        for (auto &centroid : valid_targets)
        {
            pcl::PointXYZRGB pt;
            pt.x = centroid.x;
            pt.y = centroid.y;
            pt.z = centroid.z;
            pt.r = (color_idx * color_step) % 256;
            pt.g = ((color_idx * color_step) / 2) % 256;
            pt.b = 255 - (color_idx * color_step) % 256;
            cloud.points.push_back(pt);
            color_idx++;
        }

        if (!cloud.points.empty())
        {
            cloud.width = cloud.points.size();
            cloud.height = 1;
            cloud.is_dense = true;
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(cloud, output);
            output.header.frame_id = frame_;
            output.header.stamp = ros::Time::now();
            pc_pub_.publish(output);
        }

        // Best path publish
        if (!best_path.empty())
        {
            geometry_msgs::PointStamped msg;
            msg.header.frame_id = frame_;
            msg.header.stamp = ros::Time::now();
            msg.point.x = best_target.x;
            msg.point.y = best_target.y;
            msg.point.z = best_target.z;
            target_pub_.publish(msg);

            ROS_INFO("Best target published with path length = %.2f (m)", best_cost);
        }
    }

    bool aStarPath(const Vec3 &start, const Vec3 &goal, std::vector<Vec3> &path)
    {
        if (!tree_)
            return false;

        // Octomap index dönüşümü için voxel boyutu
        double resolution = tree_->getResolution();

        auto heuristic = [&](const octomap::point3d &a, const octomap::point3d &b)
        {
            return (a - b).norm(); // Euclidean distance
        };

        octomap::point3d start_p(start.x, start.y, start.z);
        octomap::point3d goal_p(goal.x, goal.y, goal.z);

        struct Node
        {
            octomap::point3d pos;
            double g, h;
            Node *parent;
            Node(const octomap::point3d &p, double g_, double h_, Node *par = nullptr)
                : pos(p), g(g_), h(h_), parent(par) {}
            double f() const { return g + h; }
        };

        auto cmp = [](Node *a, Node *b)
        { return a->f() > b->f(); };
        std::priority_queue<Node *, std::vector<Node *>, decltype(cmp)> open_set(cmp);
        std::unordered_map<std::string, Node *> all_nodes;

        auto key = [](const octomap::point3d &p)
        {
            return std::to_string((int)(p.x() * 1000)) + "_" + std::to_string((int)(p.y() * 1000)) + "_" + std::to_string((int)(p.z() * 1000));
        };

        Node *start_node = new Node(start_p, 0.0, heuristic(start_p, goal_p));
        open_set.push(start_node);
        all_nodes[key(start_p)] = start_node;

        // 26 komşu
        std::vector<octomap::point3d> neighbors;
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                    if (dx != 0 || dy != 0 || dz != 0)
                        neighbors.emplace_back(dx * resolution, dy * resolution, dz * resolution);

        while (!open_set.empty())
        {
            Node *current = open_set.top();
            open_set.pop();

            if ((current->pos - goal_p).norm() <= resolution) // hedef voxel’e ulaştı
            {
                path.clear();
                for (Node *n = current; n; n = n->parent)
                    path.push_back({n->pos.x(), n->pos.y(), n->pos.z()});
                std::reverse(path.begin(), path.end());

                for (auto &p : all_nodes)
                    delete p.second;
                return true;
            }

            for (auto &d : neighbors)
            {
                octomap::point3d np = current->pos + d;
                octomap::OcTreeNode *node = tree_->search(np);
                if (!node || tree_->isNodeOccupied(node)) // dolu voxel
                    continue;

                double ng = current->g + d.norm();
                std::string nk = key(np);
                if (all_nodes.find(nk) == all_nodes.end() || ng < all_nodes[nk]->g)
                {
                    Node *neighbor = new Node(np, ng, heuristic(np, goal_p), current);
                    open_set.push(neighbor);
                    all_nodes[nk] = neighbor;
                }
            }
        }

        for (auto &p : all_nodes)
            delete p.second;
        return false;
    }

    int frontier_min_unknown_neighbors_ = 9;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_voxel_cc_rviz_node");
    ros::NodeHandle nh("~");

    double voxel_size, visited_radius;
    std::string frame_id;
    int min_voxels;
    double xmin_allowed, xmax_allowed, ymin_allowed, ymax_allowed;

    nh.param("voxel_size", voxel_size, 0.20);
    nh.param("visited_radius", visited_radius, 1.5); // yeni parametre
    nh.param("frame_id", frame_id, std::string("map"));
    nh.param("min_voxels_per_group", min_voxels, 10);
    nh.param("xmin_allowed", xmin_allowed, -14.0);
    nh.param("xmax_allowed", xmax_allowed, -6.0);
    nh.param("ymin_allowed", ymin_allowed, -10.0);
    nh.param("ymax_allowed", ymax_allowed, 0.0);

    // xmin_allowed = xmin_allowed + 1.0;
    // xmax_allowed = xmax_allowed - 1.0;
    // ymin_allowed = ymin_allowed + 1.0;
    // ymax_allowed = ymax_allowed - 1.0;

    OctoVoxelCCRViz node(voxel_size, frame_id, min_voxels, xmin_allowed, xmax_allowed, ymin_allowed, ymax_allowed, visited_radius);

    ros::Subscriber sub = nh.subscribe("/octomap_full_inflated", 1, &OctoVoxelCCRViz::octomapCallback, &node);

    ROS_INFO("OctoMap -> Filtered Voxel CC with A* Path Node started");

    ros::spin();
    return 0;
}
