#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <geometry_msgs/PoseStamped.h>

#include <mutex>
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <thread>

// ---------------- VOXEL TANIM -----------------
struct Voxel
{
    int x, y, z;
    bool operator==(const Voxel &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

namespace std
{
    template <>
    struct hash<Voxel>
    {
        size_t operator()(const Voxel &v) const
        {
            return ((std::hash<int>()(v.x) ^ (std::hash<int>()(v.y) << 1)) >> 1) ^ (std::hash<int>()(v.z) << 1);
        }
    };
}

// ---------------- NODE TANIM -----------------
struct Node
{
    Voxel voxel;
    double g_cost;
    double f_cost;
    Voxel parent;

    bool operator>(const Node &other) const
    {
        return f_cost > other.f_cost;
    }
};

// ---------------- A* PLANNER -----------------
class AStarPlanner
{
public:
    AStarPlanner(ros::NodeHandle &nh)
    {
        octomap_sub_ = nh.subscribe("/octomap_full_inflated", 1, &AStarPlanner::octomapCallback, this);
        odom_sub_ = nh.subscribe("/firefly/odometry_sensor1/odometry", 1, &AStarPlanner::odomCallback, this);
        target_sub_ = nh.subscribe("/cc_targets", 1, &AStarPlanner::targetCallback, this);

        path_pub_ = nh.advertise<nav_msgs::Path>("/planned_path", 1);
        planning_timer_ = nh.createTimer(ros::Duration(0.5), &AStarPlanner::timerCallback, this);

        ROS_INFO("A* Path Planner node started.");
    }

private:
    ros::Subscriber octomap_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber target_sub_;
    ros::Publisher path_pub_;
    ros::Timer planning_timer_;

    std::mutex map_mutex_;
    std::mutex state_mutex_;

    bool continue_planning_ = true;
    geometry_msgs::Point last_planning_pos_;
    double position_threshold_ = 0.25;

    octomap::OcTree *map_ = nullptr;
    geometry_msgs::Point current_pos_;
    geometry_msgs::Point goal_pos_;
    geometry_msgs::Point start_pos_;

    bool have_odom_ = false;
    bool have_goal_ = false;
    bool start_saved_ = false;
    bool returning_home_ = false;

    ros::Time last_target_time_;

    Voxel last_start_voxel, last_goal_voxel;
    bool initial_run = true;

    // ---------------- CALLBACKS -----------------
    void octomapCallback(const octomap_msgs::OctomapConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        octomap::AbstractOcTree *tree = octomap_msgs::fullMsgToMap(*msg);
        if (!tree)
        {
            ROS_WARN("Failed to convert Octomap message.");
            return;
        }
        octomap::OcTree *octree = dynamic_cast<octomap::OcTree *>(tree);
        if (!octree)
        {
            ROS_WARN("Octomap is not an OcTree.");
            delete tree;
            return;
        }

        if (map_)
            delete map_;
        map_ = octree;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_pos_ = msg->pose.pose.position;
        have_odom_ = true;

        // Sadece drone belirli bir mesafe değiştiyse planlamayı iptal et
        double dx = current_pos_.x - last_planning_pos_.x;
        double dy = current_pos_.y - last_planning_pos_.y;
        double dz = current_pos_.z - last_planning_pos_.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist > position_threshold_)
        {
            continue_planning_ = false;
        }

        // Eğer geri dönüyorsak → hedefe ulaşıldı mı kontrol et
        if (returning_home_)
        {
            double dx = current_pos_.x - goal_pos_.x;
            double dy = current_pos_.y - goal_pos_.y;
            double dz = current_pos_.z - goal_pos_.z;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (dist < position_threshold_)
            {
                ROS_WARN("Reached start position → shutting down node.");
                ros::shutdown();
            }
        }

        // ilk odom geldiğinde başlangıç konumunu kaydet
        if (!start_saved_)
        {
            start_pos_ = current_pos_;
            start_saved_ = true;
            ROS_INFO("Start position saved: (%.2f, %.2f, %.2f)", start_pos_.x, start_pos_.y, start_pos_.z);
        }
    }

    void targetCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        Voxel goal_voxel = worldToVoxel(map_, msg->point);
        Voxel free_goal_voxel = findNearestFreeVoxel(goal_voxel);
        goal_pos_ = voxelToWorld(map_, free_goal_voxel);

        have_goal_ = true;

        // Hedef değişti → planlama iptal
        continue_planning_ = false;

        // 🔹 cc_targets son mesaj zamanını güncelle
        last_target_time_ = ros::Time::now();
    }

    void timerCallback(const ros::TimerEvent &event)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!have_odom_ || !map_)
            return;

        // 🔹 cc_targets’tan 1 dakikadır mesaj yoksa → başlangıca dön ve node’u kapat
        if (have_goal_ && !last_target_time_.isZero())
        {
            if ((ros::Time::now() - last_target_time_).toSec() > 60.0)
            {
                ROS_WARN("No target for 60s → Returning to start position and shutting down.");

                // 🔹 start_pos_ odom’dan alınmıştı, şimdi free voxel’e en yakın noktayı bul
                Voxel goal_voxel = worldToVoxel(map_, start_pos_);
                Voxel free_goal_voxel = findNearestFreeVoxel(goal_voxel);
                goal_pos_ = voxelToWorld(map_, free_goal_voxel);

                planPath();

                returning_home_ = true;
                // return;
            }
        }

        if (have_goal_)
        {
            planPath();
        }
    }

    // ---------------- HELPERS -----------------
    Voxel worldToVoxel(const octomap::OcTree *tree, const geometry_msgs::Point &pt)
    {
        double res = tree->getResolution();
        return Voxel{static_cast<int>(std::round(pt.x / res)),
                     static_cast<int>(std::round(pt.y / res)),
                     static_cast<int>(std::round(pt.z / res))};
    }

    geometry_msgs::Point voxelToWorld(const octomap::OcTree *tree, const Voxel &v)
    {
        double res = tree->getResolution();
        geometry_msgs::Point p;
        p.x = v.x * res;
        p.y = v.y * res;
        p.z = v.z * res;
        return p;
    }

    double heuristic(const Voxel &a, const Voxel &b)
    {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
    }

    // --- 26 komşuluk ---
    std::vector<Voxel> getNeighbors(const Voxel &v)
    {
        std::vector<Voxel> neighbors;
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dz = -1; dz <= 1; ++dz)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;
                    Voxel nb{v.x + dx, v.y + dy, v.z + dz};

                    // --- Z alt sınırı: 1 metre ---
                    double res = map_->getResolution();
                    double world_z = nb.z * res;
                    if (world_z < 0.5)
                        continue;

                    neighbors.push_back(nb);
                }
            }
        }
        return neighbors;
    }

    // --- Hareket maliyeti ---
    double moveCost(const Voxel &a, const Voxel &b)
    {
        int dx = std::abs(a.x - b.x);
        int dy = std::abs(a.y - b.y);
        int dz = std::abs(a.z - b.z);
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Path'in ilk kordinatı ile benim konumum yakın mı?
    bool isPathValid(const std::vector<geometry_msgs::PoseStamped> &path)
    {
        if (path.empty())
            return false;

        const auto &first_pose = path.front().pose.position;

        double dx = current_pos_.x - first_pose.x;
        double dy = current_pos_.y - first_pose.y;
        double dz = current_pos_.z - first_pose.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        return (dist <= 0.25); // 25 cm eşik
    }

    Voxel findNearestFreeVoxel(const Voxel &start_voxel)
    {
        if (!map_)
            return start_voxel;

        std::queue<Voxel> q;
        std::unordered_map<Voxel, bool> visited;
        q.push(start_voxel);
        visited[start_voxel] = true;

        while (!q.empty())
        {
            Voxel v = q.front();
            q.pop();

            geometry_msgs::Point p = voxelToWorld(map_, v);
            octomap::OcTreeNode *node = map_->search(p.x, p.y, p.z);

            // 🔹 free voxel bulundu
            if (node && !map_->isNodeOccupied(node))
            {
                return v;
            }

            // 🔹 komşulara bak
            for (const Voxel &nb : getNeighbors(v))
            {
                if (visited.find(nb) == visited.end())
                {
                    q.push(nb);
                    visited[nb] = true;
                }
            }
        }

        ROS_WARN("No free voxel found near start!");
        return start_voxel;
    }

    // ---------------- SMOOTHING -----------------
    std::vector<geometry_msgs::PoseStamped> smoothPath(const std::vector<geometry_msgs::PoseStamped> &path, int iterations)
    {
        if (path.size() < 3 || iterations <= 0)
        {
            return path;
        }

        std::vector<geometry_msgs::PoseStamped> current_path = path;

        for (int i = 0; i < iterations; ++i)
        {
            if (current_path.size() < 3)
                break;

            std::vector<geometry_msgs::PoseStamped> next_path;
            next_path.push_back(current_path.front());

            for (size_t j = 1; j < current_path.size() - 1; ++j)
            {
                const auto &p1 = current_path[j - 1].pose.position;
                const auto &p2 = current_path[j].pose.position;
                const auto &p3 = current_path[j + 1].pose.position;

                geometry_msgs::Point new_point1;
                new_point1.x = 0.25 * p1.x + 0.75 * p2.x;
                new_point1.y = 0.25 * p1.y + 0.75 * p2.y;
                new_point1.z = 0.25 * p1.z + 0.75 * p2.z;

                geometry_msgs::Point new_point2;
                new_point2.x = 0.75 * p2.x + 0.25 * p3.x;
                new_point2.y = 0.75 * p2.y + 0.25 * p3.y;
                new_point2.z = 0.75 * p2.z + 0.25 * p3.z;

                geometry_msgs::PoseStamped pose1, pose2;
                pose1.header = current_path[j].header;
                pose1.pose.position = new_point1;
                pose1.pose.orientation.w = 1.0;
                next_path.push_back(pose1);

                pose2.header = current_path[j].header;
                pose2.pose.position = new_point2;
                pose2.pose.orientation.w = 1.0;
                next_path.push_back(pose2);
            }

            next_path.push_back(current_path.back());
            current_path = next_path;
        }
        return current_path;
    }

    // ---------------- PLAN PATH -----------------
    void planPath()
    {
        if (!map_)
            return;

        continue_planning_ = true;
        last_planning_pos_ = current_pos_;

        std::unique_lock<std::mutex> lock(map_mutex_);
        Voxel start = worldToVoxel(map_, current_pos_);
        Voxel goal = worldToVoxel(map_, goal_pos_);

        if (!initial_run && start == last_start_voxel && goal == last_goal_voxel)
            return;

        last_start_voxel = start;
        last_goal_voxel = goal;
        initial_run = false;

        ROS_INFO("Planning path from (%d, %d, %d) to (%d, %d, %d)", start.x, start.y, start.z, goal.x, goal.y, goal.z);

        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
        std::unordered_map<Voxel, double> g_costs;
        std::unordered_map<Voxel, Voxel> came_from;

        open_set.push(Node{start, 0.0, heuristic(start, goal), start});
        g_costs[start] = 0.0;

        bool found = false;
        int max_iter = 50000;
        int iter = 0;

        while (!open_set.empty() && iter < max_iter)
        {
            if (!continue_planning_)
            {
                ROS_WARN("Path planning canceled due to new start or goal");
                return; // planlama iptal edildi
            }
            iter++;
            Node current = open_set.top();
            open_set.pop();

            if (current.voxel == goal)
            {
                found = true;
                break;
            }

            for (const Voxel &neighbor : getNeighbors(current.voxel))
            {
                geometry_msgs::Point neighbor_world = voxelToWorld(map_, neighbor);
                octomap::OcTreeNode *node = map_->search(neighbor_world.x, neighbor_world.y, neighbor_world.z);

                // 1. Eğer voxel unknown veya occupied ise geçme
                if (!node || map_->isNodeOccupied(node))
                {
                    continue;
                }

                // 2. Duvar yakınlığına göre ekstra ceza
                double clearance_penalty = 0.0;
                for (const Voxel &nb2 : getNeighbors(neighbor))
                {
                    geometry_msgs::Point p2 = voxelToWorld(map_, nb2);
                    octomap::OcTreeNode *n2 = map_->search(p2.x, p2.y, p2.z);

                    if (n2 && map_->isNodeOccupied(n2))
                    {
                        clearance_penalty += 5.0; // duvara yakınsa +5 maliyet
                    }
                }

                // 3. Normal hareket maliyeti + ceza
                double tentative_g = g_costs[current.voxel] + moveCost(current.voxel, neighbor) + clearance_penalty;

                if (g_costs.find(neighbor) == g_costs.end() || tentative_g < g_costs[neighbor])
                {
                    g_costs[neighbor] = tentative_g;
                    double f = tentative_g + heuristic(neighbor, goal);
                    open_set.push(Node{neighbor, tentative_g, f, current.voxel});
                    came_from[neighbor] = current.voxel;
                }
            }
        }
        lock.unlock();

        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "world";
        path_msg.header.stamp = ros::Time::now();

        if (!found)
        {
            ROS_WARN("Path not found! Tried %d iterations.", iter);
            path_pub_.publish(path_msg);
            return;
        }

        std::vector<geometry_msgs::PoseStamped> a_star_path;
        Voxel v = goal;
        while (!(v == start))
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position = voxelToWorld(map_, v);
            a_star_path.push_back(pose);
            if (came_from.find(v) == came_from.end())
            {
                ROS_WARN("Path reconstruction failed!");
                return;
            }
            v = came_from[v];
        }
        geometry_msgs::PoseStamped start_pose;
        start_pose.pose.position = voxelToWorld(map_, start);
        a_star_path.push_back(start_pose);
        std::reverse(a_star_path.begin(), a_star_path.end());

        // Chaikin smoothing (tek iterasyon)
        std::vector<geometry_msgs::PoseStamped> final_path = smoothPath(a_star_path, 1);

        for (const auto &pose : final_path)
        {
            path_msg.poses.push_back(pose);
        }

        if (isPathValid(final_path))
        {
            ROS_INFO("Path is valid → Publishing path.");
            path_pub_.publish(path_msg);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "a_star_planner_node");
    ros::NodeHandle nh;
    AStarPlanner planner(nh);
    ros::spin();
    return 0;
}
