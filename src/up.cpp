#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <cmath>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go_up_80_node");
    ros::NodeHandle nh("~");

    ros::Publisher trajectory_pub = nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 10);
    tf::TransformListener listener;

    // Başlangıç pozisyonunu al (sadece bir kere)
    geometry_msgs::PoseStamped start_pose;
    try
    {
        tf::StampedTransform transform;
        listener.waitForTransform("/world", "/firefly/base_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/world", "/firefly/base_link", ros::Time(0), transform);

        start_pose.header.frame_id = "world";
        start_pose.pose.position.x = transform.getOrigin().x();
        start_pose.pose.position.y = transform.getOrigin().y();
        start_pose.pose.position.z = transform.getOrigin().z();

        start_pose.pose.orientation.x = transform.getRotation().x();
        start_pose.pose.orientation.y = transform.getRotation().y();
        start_pose.pose.orientation.z = transform.getRotation().z();
        start_pose.pose.orientation.w = transform.getRotation().w();
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return -1;
    }

    // Hedef pozisyon: start x ve y, z +80
    geometry_msgs::PoseStamped target_pose = start_pose;
    target_pose.pose.position.z += 80.0;

    // Adım boyutu
    double step = 0.3; // her turda en fazla 0.3 m ilerle
    ros::Rate rate(20);

    // Mevcut pozisyon
    geometry_msgs::PoseStamped current_pose = start_pose;

    while (ros::ok())
    {
        // Hedefe olan fark
        double dx = target_pose.pose.position.x - current_pose.pose.position.x;
        double dy = target_pose.pose.position.y - current_pose.pose.position.y;
        double dz = target_pose.pose.position.z - current_pose.pose.position.z;

        double dist = sqrt(dx * dx + dy * dy + dz * dz);

        if (dist < 0.1) // hedefe ulaştı
        {
            ROS_INFO("Hedefe ulasildi!");
            break;
        }

        // normalize edip küçük adım ekle
        double step_size = std::min(step, dist);
        current_pose.pose.position.x += dx / dist * step_size;
        current_pose.pose.position.y += dy / dist * step_size;
        current_pose.pose.position.z += dz / dist * step_size;

        // Frame + zaman
        current_pose.header.frame_id = "world";
        current_pose.header.stamp = ros::Time::now();

        trajectory_pub.publish(current_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
