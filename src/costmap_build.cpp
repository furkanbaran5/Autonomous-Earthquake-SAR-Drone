#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
#include <vector>

class OctomapInflator
{
public:
    OctomapInflator(ros::NodeHandle &nh)
    {
        // Parametreleri ROS sunucusundan okuyun
        nh.param<int>("inflation_radius_voxels", inflation_radius_voxels_, 1);
        nh.param<double>("inflation_log_odds_threshold", inflation_log_odds_threshold_, 0.7);

        // Genişletme yarıçapı 1'den küçükse en az 1 olarak ayarla
        if (inflation_radius_voxels_ < 1)
        {
            inflation_radius_voxels_ = 1;
        }

        octomap_sub_ = nh.subscribe("/octomap_full", 1, &OctomapInflator::octomapCallback, this);
        inflated_map_pub_ = nh.advertise<octomap_msgs::Octomap>("/octomap_full_inflated", 1);
        inflated_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/octomap_inflated_cloud", 1);
        
        ROS_INFO("OctomapInflator node started. Publishing inflated map on /octomap_full_inflated and /octomap_inflated_cloud");
        ROS_INFO("Inflation radius: %d voxels", inflation_radius_voxels_);
        ROS_INFO("Inflation log odds threshold: %.2f", inflation_log_odds_threshold_);
    }

private:
    ros::Subscriber octomap_sub_;
    ros::Publisher inflated_map_pub_;
    ros::Publisher inflated_cloud_pub_;
    std::mutex map_mutex_;

    int inflation_radius_voxels_;
    double inflation_log_odds_threshold_;

    void octomapCallback(const octomap_msgs::OctomapConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);

        octomap::AbstractOcTree *abstract_tree = octomap_msgs::fullMsgToMap(*msg);
        if (!abstract_tree)
        {
            ROS_WARN("Failed to convert Octomap message to OcTree.");
            return;
        }

        octomap::OcTree *original_tree = dynamic_cast<octomap::OcTree *>(abstract_tree);
        if (!original_tree)
        {
            ROS_WARN("Octomap is not an OcTree.");
            delete abstract_tree;
            return;
        }

        octomap::OcTree *inflated_tree = new octomap::OcTree(*original_tree);
        
        std::vector<octomap::point3d> new_occupied_points;
        double resolution = original_tree->getResolution();
        
        for (octomap::OcTree::leaf_iterator it = original_tree->begin_leafs(), end = original_tree->end_leafs(); it != end; ++it)
        {
            // Yalnızca doluluk eşiğini aşan dolu hücreleri genişlet
            if (original_tree->isNodeOccupied(*it))
            {
                // Genişletme yarıçapı parametresini kullanma
                for (int dx = -inflation_radius_voxels_; dx <= inflation_radius_voxels_; ++dx)
                {
                    for (int dy = -inflation_radius_voxels_; dy <= inflation_radius_voxels_; ++dy)
                    {
                        for (int dz = -inflation_radius_voxels_; dz <= inflation_radius_voxels_; ++dz)
                        {
                            if (dx == 0 && dy == 0 && dz == 0) continue;
                            
                            octomap::point3d neighbor_coord(it.getX() + dx * resolution, 
                                                            it.getY() + dy * resolution, 
                                                            it.getZ() + dz * resolution);
                            
                            octomap::OcTreeNode* neighbor_node = original_tree->search(neighbor_coord);
                            
                            // Komşu hücre boş veya bilinmiyorsa, yeni dolu hücreler arasına ekle
                            if (!neighbor_node || !original_tree->isNodeOccupied(neighbor_node))
                            {
                                new_occupied_points.push_back(neighbor_coord);
                            }
                        }
                    }
                }
            }
        }
        
        // Genişletilmiş hücreleri güncelleme
        for (const auto& p : new_occupied_points)
        {
            inflated_tree->updateNode(p, true);
        }

        // Genişletilmiş haritayı OctoMap formatında yayınla
        octomap_msgs::Octomap inflated_msg;
        inflated_msg.header = msg->header;
        octomap_msgs::fullMapToMsg(*inflated_tree, inflated_msg);
        inflated_map_pub_.publish(inflated_msg);

        // Genişletilmiş haritayı PointCloud2 olarak yayınla
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        for (octomap::OcTree::leaf_iterator it = inflated_tree->begin_leafs(), end = inflated_tree->end_leafs(); it != end; ++it)
        {
            if (inflated_tree->isNodeOccupied(*it))
            {
                pcl::PointXYZRGB p;
                p.x = it.getX();
                p.y = it.getY();
                p.z = it.getZ();
                p.r = 255;
                p.g = 0;
                p.b = 0;
                cloud.points.push_back(p);
            }
        }

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.frame_id = msg->header.frame_id;
        cloud_msg.header.stamp = ros::Time::now();
        inflated_cloud_pub_.publish(cloud_msg);
        
        delete original_tree;
        delete inflated_tree;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_inflator_node");
    ros::NodeHandle nh("~"); // Özel bir ad alanı (private namespace) kullanın
    OctomapInflator inflator(nh);
    ros::spin();
    return 0;
}