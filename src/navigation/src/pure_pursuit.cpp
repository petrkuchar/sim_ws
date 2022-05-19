//
// Created by pkuchar on 03.12.21.
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


using namespace std;
using namespace ros;

class PurePursuit{
private:
    NodeHandle n;
    Subscriber point_sub;
    Publisher way_pub;
    Subscriber pose_sub;
    Publisher drive_pub;
    Publisher base_pub;
    double x, y, psi;
    double la = 0.4;
    double L = 0.1;
    sensor_msgs::PointCloud waypoints;

public:
    PurePursuit(){
        n = NodeHandle();
        way_pub = n.advertise<visualization_msgs::Marker>("/waypoint", 1000);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1000);
        point_sub = n.subscribe("/path", 10, &PurePursuit::pathCallBack, this);
        pose_sub = n.subscribe("/odom", 1000, &PurePursuit::poseCallBack, this);
    }

    void pathCallBack(const sensor_msgs::PointCloud::ConstPtr &pc) {
        waypoints.points = pc->points;
    }

    void poseCallBack(const nav_msgs::Odometry::ConstPtr &odom) {
        tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        x = odom->pose.pose.position.x;
        y = odom->pose.pose.position.y;
        psi = yaw;
        sensor_msgs::PointCloud base;
        geometry_msgs::Point32 p;
        double r1= x;
        double r2 = y;
        double c = cos(psi);
        double s = sin(psi);
        for (int i = 0; i < waypoints.points.size(); ++i) {
            p.x = waypoints.points[i].x*c - r1*c - r2*s + waypoints.points[i].y*s;
            p.y = waypoints.points[i].y*c - r2*c + r1*s - waypoints.points[i].x*s;
            p.z = 0;
            base.points.push_back(p);
        }

        double delta = 10000;
        int iMin = -1;
        for (int i = 0; i < base.points.size(); ++i) {
            if(base.points[i].x >= 0) {
                p = base.points[i];
                double l = (p.x) * (p.x) + (p.y) * (p.y);
                double val = abs(l - la);
                if (delta > val) {
                    iMin = i;
                    delta = val;
                }
            }
        }
        double angle = 0;
        if(iMin >= 0){
            p = base.points[iMin];
            angle = atan(((2*L*p.y)/la)/la);
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = p.x;
        marker.pose.position.y = p.y;
        marker.pose.position.z = 0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        way_pub.publish( marker );


        ackermann_msgs::AckermannDriveStamped msg;
        msg.header.frame_id = "pure_pursuit";
        if (angle > 0.4)
            msg.drive.steering_angle = 0.4;
        if (angle < -0.4)
            msg.drive.steering_angle = -0.4;
        else
            msg.drive.steering_angle = angle;
        msg.drive.speed = 1;
        drive_pub.publish(msg);

    }
};


int main(int argc, char **argv) {
    init(argc, argv, "pure_pursuit");
    PurePursuit pp;
    spin();
    return 0;
}
