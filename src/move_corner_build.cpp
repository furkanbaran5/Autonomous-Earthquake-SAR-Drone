#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <vector>
#include <algorithm>

geometry_msgs::Point current_position;
bool got_odom = false;

// Odom callback ile güncel konumu alıyoruz
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_position = msg->pose.pose.position;
    got_odom = true;
}

// Hedefe ulaşılıp ulaşılmadığını kontrol eden fonksiyon
bool reachedGoal(double x_goal, double y_goal, double z_goal, double threshold = 0.5)
{
    double dx = current_position.x - x_goal;
    double dy = current_position.y - y_goal;
    double dz = current_position.z - z_goal;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    return dist < threshold;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goto_xy_path_node");
    ros::NodeHandle nh("~");

    ros::Subscriber odom_sub = nh.subscribe("/firefly/odometry_sensor1/odometry", 10, odomCallback);
    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        "/firefly/command/trajectory", 10, true);

    // Parametreler
    double x, y;
    nh.param("x", x, 0.0);
    nh.param("y", y, 0.0);

    x = x + 2;
    y = y + 2;

    // Güncel konumu bekle
    ROS_INFO("Mevcut konum bekleniyor...");
    ros::Rate rate(10);
    while (ros::ok() && !got_odom)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Mevcut konum alındı: x=%.2f y=%.2f z=%.2f",
             current_position.x, current_position.y, current_position.z);

    // Waypoint listesi
    std::vector<std::vector<double>> waypoints = {
        {current_position.x, current_position.y, current_position.z}, // başlangıç
        {x, y, 80.0},                                                 // önce yükseğe
        {x, y, 1.0}                                                   // sonra hedef z
    };

    trajectory_msgs::MultiDOFJointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.header.frame_id = "world";
    traj.joint_names.push_back("base_link");

    ros::Duration time_from_start(0.0);
    double step = 0.5;         // waypoint aralığı
    double segment_time = 0.2; // her waypoint için süre

    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
        double x_start = waypoints[i][0];
        double y_start = waypoints[i][1];
        double z_start = waypoints[i][2];
        double x_end = waypoints[i + 1][0];
        double y_end = waypoints[i + 1][1];
        double z_end = waypoints[i + 1][2];

        double dx = x_end - x_start;
        double dy = y_end - y_start;
        double dz = z_end - z_start;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        int steps = std::max(1, static_cast<int>(dist / step));

        for (int s = 1; s <= steps; s++)
        {
            double alpha = static_cast<double>(s) / steps;
            double tx = x_start + alpha * dx;
            double ty = y_start + alpha * dy;
            double tz = z_start + alpha * dz;

            tf::Quaternion q;
            q.setRPY(0, 0, 0);

            trajectory_msgs::MultiDOFJointTrajectoryPoint point;
            geometry_msgs::Transform transform;
            transform.translation.x = tx;
            transform.translation.y = ty;
            transform.translation.z = tz;
            transform.rotation.x = q.x();
            transform.rotation.y = q.y();
            transform.rotation.z = q.z();
            transform.rotation.w = q.w();

            point.transforms.push_back(transform);
            point.velocities.push_back(geometry_msgs::Twist());
            point.accelerations.push_back(geometry_msgs::Twist());

            time_from_start += ros::Duration(segment_time);
            point.time_from_start = time_from_start;

            traj.points.push_back(point);
        }
    }

    // Subscriber bekle
    while (traj_pub.getNumSubscribers() < 1 && ros::ok())
    {
        ROS_INFO("Trajectory subscriber bekleniyor...");
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("Trajectory gönderiliyor. Toplam %lu waypoint var.", traj.points.size());
    traj_pub.publish(traj);

    // Hedefe ulaşana kadar dön
    ROS_INFO("Hedefe ulaşılması bekleniyor...");
    while (ros::ok() && !reachedGoal(x, y, 1.0))
    {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Hedefe ulaşıldı. Node kapanıyor.");
    return 0;
}
