#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <tf/tf.h>
#include <vector>
#include <cmath>

// --- Global değişkenler ---
bool is_executing_trajectory = false;
geometry_msgs::Point final_target;
std::vector<geometry_msgs::Point> path_points;
bool is_waiting = false;
ros::Time reached_time;
double current_yaw = 0.0;
bool path_available = false;
geometry_msgs::Point current_pos;

// Publisher global
ros::Publisher traj_pub;

// === TRAJECTORY OLUŞTURMA ===
trajectory_msgs::MultiDOFJointTrajectory generateTrajectory()
{
    trajectory_msgs::MultiDOFJointTrajectory traj_msg;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = "map";

    const double yaw_smooth = 0.3;
    double prev_yaw = 0.0;

    double time_from_start = 0.0;
    double max_speed = 1.0;
    double dt = 0.1;

    if (path_points.empty())
        return traj_msg;

    geometry_msgs::Point prev_point = path_points[0];

    // === Başlangıç yönünü ayarlama ===
    if (path_points.size() >= 2)
    {
        geometry_msgs::Point start_point = path_points[0];
        geometry_msgs::Point next_point = path_points[1];

        double dx = next_point.x - start_point.x;
        double dy = next_point.y - start_point.y;
        double init_yaw = atan2(dy, dx); // hedef yön

        double yaw_diff = init_yaw - current_yaw;
        while (yaw_diff > M_PI)
            yaw_diff -= 2 * M_PI;
        while (yaw_diff < -M_PI)
            yaw_diff += 2 * M_PI;

        double align_duration = 5.0;
        int align_steps = 20;

        for (int i = 0; i <= align_steps; i++)
        {
            double yaw = current_yaw + (yaw_diff * i) / align_steps;

            geometry_msgs::Transform tf_msg;
            tf_msg.translation.x = start_point.x;
            tf_msg.translation.y = start_point.y;
            tf_msg.translation.z = start_point.z;

            tf::Quaternion q;
            q.setRPY(0, 0, yaw);
            tf_msg.rotation.x = q.x();
            tf_msg.rotation.y = q.y();
            tf_msg.rotation.z = q.z();
            tf_msg.rotation.w = q.w();

            trajectory_msgs::MultiDOFJointTrajectoryPoint point;
            point.transforms.push_back(tf_msg);
            point.velocities.push_back(geometry_msgs::Twist());
            point.accelerations.push_back(geometry_msgs::Twist());
            point.time_from_start = ros::Duration(time_from_start);
            time_from_start += align_duration / align_steps;

            traj_msg.points.push_back(point);
        }
        prev_yaw = init_yaw;
    }

    // === Path boyunca ilerleme ===
    for (size_t i = 0; i < path_points.size(); ++i)
    {
        geometry_msgs::Point target = path_points[i];

        double dx = target.x - prev_point.x;
        double dy = target.y - prev_point.y;
        double dz = target.z - prev_point.z;
        double dist = sqrt(dx * dx + dy * dy + dz * dz);
        double target_yaw = atan2(dy, dx);

        double yaw_diff = target_yaw - prev_yaw;
        while (yaw_diff > M_PI)
            yaw_diff -= 2 * M_PI;
        while (yaw_diff < -M_PI)
            yaw_diff += 2 * M_PI;
        double yaw = prev_yaw + yaw_smooth * yaw_diff;
        prev_yaw = yaw;

        double segment_time = dist / max_speed;
        if (segment_time < 1e-3)
            segment_time = dt;

        geometry_msgs::Transform tf_msg;
        tf_msg.translation.x = target.x;
        tf_msg.translation.y = target.y;
        tf_msg.translation.z = target.z + 0.25;

        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        tf_msg.rotation.x = q.x();
        tf_msg.rotation.y = q.y();
        tf_msg.rotation.z = q.z();
        tf_msg.rotation.w = q.w();

        trajectory_msgs::MultiDOFJointTrajectoryPoint point;
        point.transforms.push_back(tf_msg);
        point.velocities.push_back(geometry_msgs::Twist());
        point.accelerations.push_back(geometry_msgs::Twist());

        time_from_start += segment_time;
        point.time_from_start = ros::Duration(time_from_start);

        traj_msg.points.push_back(point);
        prev_point = target;
    }

    // === Hedefte yükselme + 360° dönüş ===
    final_target = path_points.back();

    double climb_height = 0.4;
    double climb_duration = 3.0;
    int climb_steps = 10;

    for (int i = 1; i <= climb_steps; i++)
    {
        geometry_msgs::Transform tf_msg;
        tf_msg.translation.x = final_target.x;
        tf_msg.translation.y = final_target.y;
        tf_msg.translation.z = final_target.z + (climb_height * i / climb_steps);

        tf::Quaternion q;
        q.setRPY(0, 0, prev_yaw);
        tf_msg.rotation.x = q.x();
        tf_msg.rotation.y = q.y();
        tf_msg.rotation.z = q.z();
        tf_msg.rotation.w = q.w();

        trajectory_msgs::MultiDOFJointTrajectoryPoint point;
        point.transforms.push_back(tf_msg);
        point.velocities.push_back(geometry_msgs::Twist());
        point.accelerations.push_back(geometry_msgs::Twist());
        point.time_from_start = ros::Duration(time_from_start);
        time_from_start += climb_duration / climb_steps;

        traj_msg.points.push_back(point);
    }

    double spin_duration = 10.0;
    int spin_steps = 36;

    for (int i = 0; i <= spin_steps; i++)
    {
        double yaw = prev_yaw + (2 * M_PI * i) / spin_steps;

        geometry_msgs::Transform tf_msg;
        tf_msg.translation.x = final_target.x;
        tf_msg.translation.y = final_target.y;
        tf_msg.translation.z = final_target.z + climb_height;

        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        tf_msg.rotation.x = q.x();
        tf_msg.rotation.y = q.y();
        tf_msg.rotation.z = q.z();
        tf_msg.rotation.w = q.w();

        trajectory_msgs::MultiDOFJointTrajectoryPoint point;
        point.transforms.push_back(tf_msg);
        point.velocities.push_back(geometry_msgs::Twist());
        point.accelerations.push_back(geometry_msgs::Twist());
        point.time_from_start = ros::Duration(time_from_start);
        time_from_start += spin_duration / spin_steps;

        traj_msg.points.push_back(point);
    }

    return traj_msg;
}

// === CALLBACKS ===
void pathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    if (msg->poses.size() == 0)
    {
        ROS_WARN("Path has less than 0 points. Waiting for more points...");
        path_points.clear();
        path_available = false;
        return;
    }

    path_points.clear();
    for (const auto &pose : msg->poses)
        path_points.push_back(pose.pose.position);

    path_available = true;
    ROS_INFO("Received path with %lu points", path_points.size());
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_pos = msg->pose.pose.position;

    if (is_executing_trajectory && !path_points.empty())
    {
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch;
        tf::Matrix3x3(q).getRPY(roll, pitch, current_yaw);

        double dx = msg->pose.pose.position.x - final_target.x;
        double dy = msg->pose.pose.position.y - final_target.y;
        double dz = msg->pose.pose.position.z - final_target.z;
        double dist_to_target = sqrt(dx * dx + dy * dy + dz * dz);

        if (dist_to_target < 0.25 && !is_waiting)
        {
            ROS_INFO("Reached target. Starting 15s wait...");
            is_executing_trajectory = false;
            is_waiting = true;
            reached_time = ros::Time::now();
        }
    }
}

bool isPathValid()
{
    if (path_points.empty())
        return false;

    geometry_msgs::Point first_point = path_points.front();

    double dx = current_pos.x - first_point.x;
    double dy = current_pos.y - first_point.y;
    double dz = current_pos.z - first_point.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    return (dist <= 0.25); // 25 cm eşik
}

// === MAIN ===
int main(int argc, char **argv)
{
    ros::init(argc, argv, "firefly_path_to_trajectory");
    ros::NodeHandle nh;

    traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        "/firefly/command/trajectory", 10);

    ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>(
        "/planned_path", 1, pathCallback);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "/firefly/odometry_sensor1/odometry", 10, odomCallback);

    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        // Eğer bekleme modundaysak süreyi kontrol et (trajectory publish ETME)
        if (is_waiting)
        {
            if ((ros::Time::now() - reached_time).toSec() >= 15.0)
            {
                ROS_INFO("15s wait finished.");
                is_waiting = false;
            }
        }

        // Normal publish akışı: sadece yeni path geldiğinde çalışır
        if (!is_executing_trajectory && !is_waiting && path_available)
        {
            if (isPathValid())
            {
                trajectory_msgs::MultiDOFJointTrajectory traj_msg = generateTrajectory();
                traj_pub.publish(traj_msg);
                is_executing_trajectory = true;

                path_available = false; // eski path tekrar kullanılmaz
                ROS_INFO("Published trajectory with %lu points", traj_msg.points.size());
            }
            else
            {
                ROS_WARN("Trajectory rejected: Drone too far from path start.");
                path_available = false; // bu path’i de iptal et
            }
        }

        rate.sleep();
    }

    return 0;
}
