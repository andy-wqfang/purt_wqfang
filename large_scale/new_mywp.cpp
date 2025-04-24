#include "./include/traj_min_jerk.hpp"
#include "./include/traj_min_snap.hpp"

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <vector>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace min_jerk;
using namespace min_snap;

VectorXd allocateTime(const MatrixXd &wayPs, double max_vel)
{
    int N = wayPs.cols() - 1;
    VectorXd durations(N);
    for (int k = 0; k < N; k++)
    {
        double dist = (wayPs.col(k + 1) - wayPs.col(k)).norm();
        durations(k) = dist / max_vel;
    }
    return durations;
}

void writeTrajectoryToCSV(const string &filename, min_jerk::Trajectory &traj, double step = 0.1)
{
    ofstream out(filename);
    out << "t,x,y,z\n";
    double t_total = traj.getTotalDuration();
    for (double t = 0.0; t <= t_total; t += step)
    {
        Vector3d pos = traj.getPos(t);
        out << t << "," << pos.x() << "," << pos.y() << "," << pos.z() << "\n";
    }
    out.close();
}

void writeTrajectoryToCSV(const string &filename, min_snap::Trajectory &traj, double step = 0.1)
{
    ofstream out(filename);
    out << "t,x,y,z\n";
    double t_total = traj.getTotalDuration();
    for (double t = 0.0; t <= t_total; t += step)
    {
        Vector3d pos = traj.getPos(t);
        out << t << "," << pos.x() << "," << pos.y() << "," << pos.z() << "\n";
    }
    out.close();
}

int main()
{
    double alt = 3;
    double x0 = 3.2;
    double y0 = 0.0;
    double xf = 10.0;
    double yf = -20.0;

    vector<Vector3d> control_points = {
        {x0, y0, 0.0},
        {35.0, 0.0, alt},
        {45.0, 0.0, alt},
        {45.0, 25.0, alt},
        {-8.0, 25.0, alt},
        {-10.0, 1.0, alt},
        {5.0, -15.0, 2.0},
        {xf, yf, 0.1}
    };

    MatrixXd route(3, control_points.size());
    for (int i = 0; i < control_points.size(); ++i)
        route.col(i) = control_points[i];

    int N = control_points.size() - 1;

    Matrix3d iS = Matrix3d::Zero();
    Matrix3d fS = Matrix3d::Zero();
    iS.col(0) = control_points.front();
    fS.col(0) = control_points.back();

    Matrix<double, 3, 4> iSS, fSS;
    iSS << iS.col(0), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero();
    fSS << fS.col(0), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero();

    VectorXd ts = allocateTime(route, 3.0);

    min_jerk::JerkOpt jerkOpt;
    min_jerk::Trajectory jerkTraj;
    jerkOpt.reset(iS, fS, N);
    jerkOpt.generate(route.block(0, 1, 3, N - 1), ts);
    jerkOpt.getTraj(jerkTraj);
    writeTrajectoryToCSV("min_jerk_traj.csv", jerkTraj);

    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory snapTraj;
    snapOpt.reset(iSS, fSS, N);
    snapOpt.generate(route.block(0, 1, 3, N - 1), ts);
    snapOpt.getTraj(snapTraj);
    writeTrajectoryToCSV("min_snap_traj.csv", snapTraj);

    cout << "Trajectories exported to CSV." << endl;
    return 0;
}
