#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turn_360_node");
    ros::NodeHandle nh("~");

    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        "/firefly/command/trajectory", 10, true); // latched=true

    // Parametreler
    double x0, y0, z, x1, y1;
    nh.param("x0", x0, -6.0);
    nh.param("y0", y0, 0.0);
    nh.param("x1", x1, -14.0);
    nh.param("y1", y1, -10.0);
    nh.param("z", z, 2.0);

    // Orta nokta
    double cx = (x0 + x1) / 2.0;
    double cy = (y0 + y1) / 2.0;

    // Dikdörtgen köşeleri
    std::vector<std::vector<double>> corners = {
        {x0 + 1, y0 + 1, z},
        {x0 + 1, y1 - 1, z},
        {x1 - 1, y1 - 1, z},
        {x1 - 1, y0 + 1, z},
        {x0 + 1, y0 + 1, z} // başlangıç
    };

    trajectory_msgs::MultiDOFJointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.header.frame_id = "world";
    traj.joint_names.push_back("base_link");

    ros::Duration time_from_start(0.0);

    double step = 0.25;        // waypoint aralığı [m]
    double segment_time = 0.2; // her 0.25 m için tahmini süre

    for (size_t i = 0; i < corners.size() - 1; i++)
    {
        double x_start = corners[i][0];
        double y_start = corners[i][1];
        double z_start = corners[i][2];
        double x_end = corners[i + 1][0];
        double y_end = corners[i + 1][1];
        double z_end = corners[i + 1][2];

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

            // yaw merkeze bakacak
            double dx_c = cx - tx;
            double dy_c = cy - ty;
            double yaw = atan2(dy_c, dx_c);

            tf::Quaternion q;
            q.setRPY(0, 0, yaw);

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
            point.velocities.push_back(geometry_msgs::Twist());    // boş hız
            point.accelerations.push_back(geometry_msgs::Twist()); // boş ivme

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

    // Trajectory yayınla
    traj_pub.publish(traj);
    ROS_INFO("Trajectory yayınlandı. Toplam %lu nokta var.", traj.points.size());

    // Trajectory tamamlanmasını bekle
    if (!traj.points.empty())
    {
        ros::Duration total_time = traj.points.back().time_from_start + ros::Duration(1.0); // 1 sn güvenlik payı
        ROS_INFO("360 derece tur tamamlanmasını bekleniyor (%.2f sn)...", total_time.toSec());
        total_time.sleep();
    }

    ROS_INFO("360 derece tur tamamlandı, node kapanıyor.");
    return 0;
}
