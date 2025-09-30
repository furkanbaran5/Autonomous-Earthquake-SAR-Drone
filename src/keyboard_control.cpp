#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <tf/transform_listener.h>

// Tuş girdilerini yakalamak için fonksiyon
int getch()
{
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_trajectory_node");
    ros::NodeHandle nh;

    ros::Publisher trajectory_pub = nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 10);
    tf::TransformListener listener;

    double step_size = 0.3; // metre
    double yaw_step = M_PI / 6.0; // radyan

    ROS_INFO("Drone kontrol node'u baslatildi.");
    ROS_INFO("---------------------------");
    ROS_INFO("Kontrol tuslari:");
    ROS_INFO("W/S: ileri/geri");
    ROS_INFO("A/D: sola/saga");
    ROS_INFO("Q/E: sola/saga donus");
    ROS_INFO("U/J: yukselme/alcalma");
    ROS_INFO("ESC: cikis");
    ROS_INFO("---------------------------");

    ros::Rate rate(100); // 100 Hz

    // Başlangıç pozisyonu
    geometry_msgs::PoseStamped target_pose;
    bool initialized = false;

    while (ros::ok())
    {
        if (!initialized)
        {
            try
            {
                tf::StampedTransform transform;
                listener.waitForTransform("/world", "/firefly/base_link", ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform("/world", "/firefly/base_link", ros::Time(0), transform);

                target_pose.header.frame_id = "world";
                target_pose.pose.position.x = transform.getOrigin().x();
                target_pose.pose.position.y = transform.getOrigin().y();
                target_pose.pose.position.z = transform.getOrigin().z();
                target_pose.pose.orientation.x = transform.getRotation().x();
                target_pose.pose.orientation.y = transform.getRotation().y();
                target_pose.pose.orientation.z = transform.getRotation().z();
                target_pose.pose.orientation.w = transform.getRotation().w();

                initialized = true;
                ROS_INFO("Baslangic pozisyonu alindi.");
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
                continue;
            }
        }

        target_pose.header.stamp = ros::Time::now();

        int c = getch();

        double current_yaw = tf::getYaw(target_pose.pose.orientation); // Hedef yaw üzerinden al
        double new_yaw = current_yaw;
        double delta_x = 0;
        double delta_y = 0;
        double delta_z = 0;

        switch (c)
        {
            case 'w':
                delta_x = step_size * cos(current_yaw);
                delta_y = step_size * sin(current_yaw);
                break;
            case 's':
                delta_x = -step_size * cos(current_yaw);
                delta_y = -step_size * sin(current_yaw);
                break;
            case 'a':
                delta_x = -step_size * sin(current_yaw);
                delta_y = step_size * cos(current_yaw);
                break;
            case 'd':
                delta_x = step_size * sin(current_yaw);
                delta_y = -step_size * cos(current_yaw);
                break;
            case 'q':
                new_yaw += yaw_step;
                break;
            case 'e':
                new_yaw -= yaw_step;
                break;
            case 'u':
                delta_z += step_size;
                break;
            case 'j':
                delta_z -= step_size;
                break;
            case 27: // ESC
                ROS_INFO("Cikis yapiliyor...");
                return 0;
            default:
                continue; // Herhangi bir tusa basilmazsa devam et
        }

        // Yeni hedef pozisyonu
        target_pose.pose.position.x += delta_x;
        target_pose.pose.position.y += delta_y;
        target_pose.pose.position.z += delta_z;

        // Yeni hedef oryantasyonu
        tf::Quaternion q_new;
        q_new.setRPY(0, 0, new_yaw);
        target_pose.pose.orientation.x = q_new.x();
        target_pose.pose.orientation.y = q_new.y();
        target_pose.pose.orientation.z = q_new.z();
        target_pose.pose.orientation.w = q_new.w();

        trajectory_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
