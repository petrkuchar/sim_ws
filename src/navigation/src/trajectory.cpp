//
// Created by pkuchar on 29.12.21.
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include "Eigen/Dense"
#include "Eigen/QR"
#include "Eigen/Sparse"




using namespace std;
using namespace ros;


class Trajectory{
private:
    NodeHandle n;
    Subscriber path_sub;
    Subscriber pose_sub;
    sensor_msgs::PointCloud global_path;
    Publisher path_pub;
    double L = 0.3302;
    double v_max = 3;
    int N = 20;
public:
    Trajectory(){
        n = NodeHandle();
        path_pub = n.advertise<nav_msgs::Path>("/references", 1000);
        path_sub = n.subscribe("/path", 1000, &Trajectory::pathCallBack, this);
        pose_sub = n.subscribe("/odom", 1000, &Trajectory::poseCallBack, this);
    }
    void pathCallBack(const sensor_msgs::PointCloud::ConstPtr &pc) {
        global_path.points = pc->points;
    }


    double euclidian(double x1, double y1, double x2, double y2) {
        return sqrt((x1- x2)*(x1- x2) + (y1- y2)*(y1- y2));
    }

    Eigen::VectorXd bezier(double t, Eigen::Vector4d x, Eigen::Vector4d y){ // Bezierova krivka
        Eigen::VectorXd xt(6);
        double t2 = t * t;
        double t3 = t * t * t;
        xt(0) = t3*x(3) + (3*t2 - 3*t3)*x(2) + (3*t3 - 6*t2 + 3*t)*x(1) + (3*t2 - t3 - 3*t + 1)*x(0);
        xt(1) = t3*y(3) + (3*t2 - 3*t3)*y(2) + (3*t3 - 6*t2 + 3*t)*y(1) + (3*t2 - t3 - 3*t + 1)*y(0);
        xt(2) = 3*(t2-2*t+1)*(x(1)-x(0)) + 6*(1-t)*t*(x(2)-x(1))+3*t2*(x(3)-x(2));
        xt(3) = 3*(t2-2*t+1)*(y(1)-y(0)) + 6*(1-t)*t*(y(2)-y(1))+3*t2*(y(3)-y(2));
        xt(4) = 6*(1-t)*(x(2)-2*x(1)+x(0)) + 6*t*(x(3)-2*x(2)+x(1));
        xt(5) = 6*(1-t)*(y(2)-2*y(1)+y(0)) + 6*t*(y(3)-2*y(2)+y(1));
        return xt;
    }

    void poseCallBack(const nav_msgs::Odometry::ConstPtr &odom) {
        tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,
                         odom->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double x0 = odom->pose.pose.position.x;
        double y0 = odom->pose.pose.position.y;
        double psi0 = yaw;
        geometry_msgs::Point32 p;
        sensor_msgs::PointCloud ref_path;
        int min_idx = 0;
        double d_min = INT_MAX;
        double distance = 0;
        double c = cos(psi0);
        double s = sin(psi0);

        // nalezeni nejblizsiho bodu
        for (int i = 0; i < global_path.points.size(); ++i) {
            double xm = -c * (x0 - global_path.points[i].x) - s * (y0 - global_path.points[i].y);
            double ym = s*(x0 - global_path.points[i].x) + c*(-y0 + global_path.points[i].y);
            distance = euclidian(xm, ym, 0, 0);
            if (distance < d_min  && xm >= 0) {
                d_min = distance;
                min_idx = i;
            }
        }
        vector<double> xt;
        vector<double> yt;

        geometry_msgs::PoseStamped poseStamped;
        nav_msgs::Path spline;

        // vybrani bodu pro prolozeni
        if (global_path.points.size() > 0){
            double d_sum = 0;
            int k = min_idx;
            int size = global_path.points.size();
            int idx = 0;
            int p_idx = 0;
            while (v_max > d_sum){
                idx = k % size;
                p_idx = (k + 1) % size;
                d_sum += euclidian(global_path.points[idx].x, global_path.points[idx].y, global_path.points[p_idx].x, global_path.points[p_idx].y);
                p.x = global_path.points[p_idx].x;
                p.y = global_path.points[p_idx].y;
                xt.push_back(p.x);
                yt.push_back(p.y);
                k++;
            }

            // prolozeni nalezenych bodu
            int Nt = xt.size();
            double t = 0;
            double pi = 3.14;
            double psi_ref = psi0;
            double Ts = (double) 1/N;
            double ds = (double) Ts*v_max;
            for (int i = 0; i < N; ++i) {
                Eigen::Vector4d xbc;
                Eigen::Vector4d ybc;

                xbc << xt[0], xt[(int)Nt/3], xt[(int)2*Nt/3], xt[Nt-1];
                ybc << yt[0], yt[(int)Nt/3], yt[(int)2*Nt/3], yt[Nt-1];
                Eigen::VectorXd xT = bezier(t, xbc, ybc);
                t += (double) Ts;
                poseStamped.pose.position.x = xT(0); // x_r
                poseStamped.pose.position.y = xT(1); // y_r
                double v_ref = sqrt(xT(2)*xT(2) + xT(3)*xT(3));
                poseStamped.pose.position.z = v_ref; //v_r
                double a1 = (xT(5)*xT(2) - xT(4)*xT(3));
                double a2 = (xT(2)*xT(2) + xT(3)*xT(3));
                double dpsi_ref = (double) a1/a2;
                psi_ref += dpsi_ref*ds; //psi_r
                poseStamped.pose.orientation.z = psi_ref;
                double delta_ref = atan((L*dpsi_ref)/v_ref); //delta_r
                poseStamped.pose.orientation.y = delta_ref;
                poseStamped.pose.orientation.w = xT(2);
                poseStamped.pose.orientation.x = xT(3);
                spline.poses.push_back(poseStamped);
            }

        }

        poseStamped.header.frame_id = "map";
        poseStamped.header.stamp = Time::now();

        spline.header.frame_id = "map";
        spline.header.stamp = Time::now();
        path_pub.publish(spline);
    }
};

int main(int argc, char **argv) {
    init(argc, argv, "reference");
    Trajectory trajectory;
    spin();
    return 0;
}

