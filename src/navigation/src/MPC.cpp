//
// Created by pkuchar on 28.01.22.
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <cmath>
#include "Eigen/QR"
#include "Eigen/Sparse"
#include "osqp.h"
#include "osqp_configure.h"
#include "Eigen/Dense"
#include "OsqpEigen/OsqpEigen.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>

using namespace ros;

using namespace Eigen;

class MPC{
private:
    NodeHandle n;
    Subscriber path_sub;
    Subscriber pose_sub;
    nav_msgs::Path ref_path;
    Publisher drive_pub;
    Subscriber scan_sub;
    Publisher bound_pub;
    double x0 = 0, y0 = 0, v0 = 0, psi0 = 0, delta0 = 0;
    double L = 0.3302;
    double T = 0.05;
    static const int N = 10, nx = 3, nu = 2;
    double v_max = 3;
public:
    MPC(){
        n = NodeHandle();
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1000);
        path_sub = n.subscribe("/references", 1000, &MPC::pathCallBack, this);
        pose_sub = n.subscribe("/odom", 1000, &MPC::poseCallBack, this);
    }
    void pathCallBack(const nav_msgs::Path::ConstPtr &p) {
        ref_path.poses = p->poses;
    }

    Matrix<double, N*nx + N*nu, N*nx + N*nu> hessianMatrix(Matrix<double, nx, nx> Q, Matrix<double, nu, nu> R){//Hessova matice
        Matrix<double, N*nx + N*nu, N*nx + N*nu> H = Matrix<double, N*nx + N*nu, N*nx + N*nu>::Zero();
        for (int i = 0; i < N; ++i) {
            H.block(i*nx, i*nx, nx, nx) = Q;
            H.block(N*nx + i*nu, N*nx + i*nu, nu, nu) = R;
        }
        return H;
    }

    Matrix<double, N*nx + N*nu, 1> gradientVector(Matrix<double, nx, nx> Q, Matrix<double, nu, nu> R, Matrix<double, N, nx + nu + nu> xr){//gradient
        Matrix<double, N*(nx+nu), 1> g = Matrix<double, N*(nx+nu), 1>::Zero();
        for (int i = 0; i < N; ++i) {
            g.block(i*nx, 0, nx, 1) = -Q*xr.block(i, 0, 1, nx).transpose();
            g.block(i*nu + N*nx, 0, nu, 1) = -R*xr.block(i, nx, 1, nu).transpose();
        }
        return g;
    }

    Matrix<double, N, nx + nu + nu> references(){ //reference
        Matrix<double, N, nx + nu + nu> ref;
        for (int i = 0; i < N; ++i) {
            ref(i, 0) = (double) ref_path.poses[i].pose.position.x; //x_r
            ref(i, 1) = (double) ref_path.poses[i].pose.position.y; //y_r
            ref(i, 2) = (double) ref_path.poses[i].pose.orientation.z; //psi_r
            ref(i, 3) = (double) ref_path.poses[i].pose.position.z; //v_r
            ref(i, 4) = (double) ref_path.poses[i].pose.orientation.y; //delta_r
            ref(i, 5) = (double) ref_path.poses[i].pose.orientation.w; //dx
            ref(i, 6) = (double) ref_path.poses[i].pose.orientation.x; //dy
        }
        return ref;
    }


    Matrix<double, N*nx, N*nx> stateMatrix(Matrix<double, N, nx + nu + nu> xr) { //Ahat
        Matrix<double, N*nx, N*nx> Ax = Matrix<double, N*nx, N*nx>::Zero();
        Matrix<double, nx, nx> A;
        for (int i = 0; i < N-1; ++i) {
            A << 1, 0, -T * xr(i+1, 3) * sin(xr(i+1, 2)),
                 0, 1,  T * xr(i+1, 3) * cos(xr(i+1, 2)),
                 0, 0,  1;
            Ax.block(i*nx, i*nx, nx, nx).diagonal() << -1, -1, -1;
            Ax.block(i*nx + nx, i*nx, nx, nx) = A;
        }
        Ax.block((N-1)*nx, (N-1)*nx, nx, nx).diagonal() << -1, -1, -1;
        return Ax;
    }

    Matrix<double, N*nx, N*nu> inputMatrix(Matrix<double, N, nx + nu + nu> xr) { //Bhat
        Matrix<double, N*nx, N*nu> Bu = Matrix<double, N*nx, N*nu>::Zero();
        Matrix<double, nx, nu> B;
        for (int i = 0; i < N-1; ++i) {
            psi_ref = xr(i+1, 2);
            v_ref = xr(i+1, 3);
            delta_ref = xr(i+1, 4);
            B << T*cos(psi_ref),                      0,
                 T*sin(psi_ref),                      0,
                 T*tan(delta_ref)/L, T*v_ref/(L*cos(delta_ref)*cos(delta_ref));
            Bu.block(i*nx + nx, i*nu, nx, nu) = B;
        }
        return Bu;
    }

    Vector3d kinematics(VectorXd r){ //kinematika
        Vector3d xk;
        xk(0) = r(0) + r(3)*cos(r(2))*T;
        xk(1) = r(1) + r(3)*sin(r(2))*T;
        xk(2) = r(2) + r(3)/L*tan(r(4))*T;
        return xk;
    }

    Vector3d lin_kinematics(VectorXd r){//linearni aproximace
        Matrix<double, nx, nx> A;
        A <<    1, 0, -T * r(3) * sin(r(2)),
                0, 1,  T * r(3) * cos(r(2)),
                0, 0,  1;
        Matrix<double, nx, nu> B;
        B << T*cos(r(2)),                      0,
                T*sin(r(2)),                      0,
                (T*tan(r(4)))/L, (T*r(3))/(L*cos(r(4))*cos(r(4)));

        Vector3d xk;

        Vector3d x;
        x << r(0), r(1), r(2);
        Vector2d u;
        u << r(3), r(4);
        xk = A*x + B*u;
        return xk;
    }

    void poseCallBack(const nav_msgs::Odometry::ConstPtr &odom) {
        tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        x0 = odom->pose.pose.position.x;
        y0 = odom->pose.pose.position.y;
        v0 = odom->twist.twist.linear.x;
        psi0 = yaw;


        Matrix<double, nx, nx> Q = Matrix<double, nx, nx>::Zero();
        Q.diagonal() << 1, 1, 0.4;
        Matrix<double, nu, nu> R = Matrix<double, nu, nu>::Zero();
        R.diagonal() << 0.1, 0.1;
        Matrix<double, N*nx + N*nu, N*nx + N*nu> H = hessianMatrix(Q, R); //

        if (ref_path.poses.size() > 0) {
            Matrix<double, N, nx + nu + nu> ref = references();
            Matrix<double, N * nx, N * nx> Ax = stateMatrix(ref);
            Matrix<double, N * nx, N * nu> Bu = inputMatrix(ref);
            Matrix<double, N*(nx+nu), 1> g = gradientVector(Q, R, ref);

            double pi = 3.14;

            Eigen::Matrix<double, nx+nu, 1> xmax;
            Eigen::Matrix<double, nx+nu, 1> xmin;

            VectorXd lin_con(N*nx);
            for (int i = 0; i < N; ++i) {
                VectorXd r(5);
                r << ref(i, 0), ref(i, 1), ref(i, 2), ref(i, 3),ref(i, 4);
                lin_con.block(i*nx, 0, nx, 1) = lin_kinematics(r) - kinematics(r);
            }


            Matrix<double, N*nx, 1> leq = Matrix<double, N*nx, 1>::Zero();
            leq(0, 0) = -x0;
            leq(1, 0) = -y0;
            leq(2, 0) = -psi0;
            for (int i = nx; i < N*nx; ++i) {
                leq(i, 0) = lin_con((i-nx));
            }
            Matrix<double, N*nx, 1> ueq = leq;
            Matrix<double, N*nx, N*(nx+nu)> Aeq;
            Aeq.block(0, 0, N*nx, N*nx) = Ax;
            Aeq.block(0, N*nx, N*nx, N*nu) = Bu;
            Matrix<double, N*(nx+nu), N*(nx+nu)> Aiq = Matrix<double, N*(nx+nu), N*(nx+nu)>::Identity();


            int kk = 0;
            for (int i = 0; i < N*nx; i+=nx) {
                Aiq(i, i) = -ref(kk, 6); //dyr
                Aiq(i, i + 1) = ref(kk, 5); //dxr
                Aiq(i+1, i) = -ref(kk, 6); //dyr
                Aiq(i+1, i + 1) = ref(kk, 5); //dxr
                kk++;
            }
            //definovani omezeni
            Matrix<double, N*nx + N*nu, 1> liq;
            Matrix<double, N*nx + N*nu, 1> uiq;
            sensor_msgs::PointCloud pc;
            geometry_msgs::Point32 p;
            double left_width = 0.5;
            double right_width = 0.5;

            // M*x < b
            for (int i = 0; i < N; ++i) {
                double xr = ref(i, 0);
                double yr = ref(i, 1);
                double dxr = ref(i, 5);
                double dyr = ref(i, 6);
                Vector2d left_tangent_p, right_tangent_p, center_p;

                center_p << xr, yr;
                right_tangent_p = center_p + right_width * Vector2d(dyr, -dxr).normalized();
                left_tangent_p  = center_p + left_width * Vector2d(-dyr, dxr).normalized();

                double C1 =  - dyr*right_tangent_p(0) + dxr*right_tangent_p(1);
                double C2 = - dyr*left_tangent_p(0) + dxr*left_tangent_p(1);


                xmax << std::max(C1, C2), OsqpEigen::INFTY, 2*pi, v_max, pi/6; // horni omezeni
                xmin << -OsqpEigen::INFTY, std::min(C1, C2), -2*pi, 0, -pi/6; // dolni omezeni
                liq.block(i*nx, 0, nx, 1) = xmin.block(0, 0, nx, 1);
                liq.block(N*nx + i*nu, 0, nu, 1) = xmin.block(nx, 0, nu, 1);
                uiq.block(i*nx, 0, nx, 1) = xmax.block(0, 0, nx, 1);
                uiq.block(N*nx + i*nu, 0, nu, 1) = xmax.block(nx, 0, nu, 1);
            }

            Matrix<double, N*nx + N*(nx+nu), N*(nx+nu)> Ac;
            Ac.block(0, 0, N*nx, N*(nx+nu)) = Aeq;
            Ac.block(N*nx, 0, N*(nx+nu), N*(nx+nu)) = Aiq;
            Matrix<double, N*nx + N*(nx+nu), 1> lc;
            lc.block(0, 0, N*nx, 1) = leq;
            lc.block(N*nx, 0, N*(nx+nu), 1) = liq;
            Matrix<double, N*nx + N*(nx+nu), 1> uc;
            uc.block(0, 0, N*nx, 1) = ueq;
            uc.block(N*nx, 0, N*(nx+nu), 1) = uiq;

            SparseMatrix<double> P;
            SparseMatrix<double> Al;
            P = H.sparseView();
            Al = Ac.sparseView();

            // nadefinovani solveru
            OsqpEigen::Solver solver;
            solver.settings()->setVerbosity(false);
            solver.settings()->setWarmStart(true);
            solver.data()->setNumberOfVariables(nx * N + nu * N);
            solver.data()->setNumberOfConstraints(2 * nx * N + nu * N);
            solver.data()->setHessianMatrix(P);
            solver.data()->setGradient(g);
            solver.data()->setLinearConstraintsMatrix(Al);
            solver.data()->setLowerBound(lc);
            solver.data()->setUpperBound(uc);
            solver.initSolver();
            Vector2d ctr;
            VectorXd QPSolution;
            solver.solveProblem();
            QPSolution = solver.getSolution();
            ctr = QPSolution.block(nx*N, 0, nu, 1);
            ackermann_msgs::AckermannDriveStamped msg;
            msg.header.frame_id = "mpc";
            double angle = ctr(1);
            msg.drive.steering_angle = angle;
            double speed = ctr(0);
            msg.drive.speed = speed;
            drive_pub.publish(msg);
            delta0 = ctr(1), v0 = ctr(0);
            pc.header.stamp = Time::now();
            pc.header.frame_id = "map";
        }
    }
};

int main(int argc, char **argv) {
    init(argc, argv, "mpc");
    MPC mpc;
    spin();
    return 0;
}
