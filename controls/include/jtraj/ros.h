#ifndef JTRAJ_ROS_H
#define JTRAJ_ROS_H

// std
#include <cmath>

// Eigen
#include <Eigen/Core>

// ROS
#include <nav_msgs/Path.h>
#include <nav_msgs/PoseStamped.h>

namespace jtraj {
namespace ros {

template<int x_idx=0, int y_idx=1, typename EigenM = Eigen::Matrix<double,2,1>>
void toPath(std::vector<EigenM> path, nav_msgs::Path& ros_path, std::string frame_id = "controls") {
    ros_path.poses.clear();
    ros_path.header.frame_id = frame_id;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.orientation.w = 1;
    for(int i = 0; i < path.size(); ++i) {
        pose.pose.position.x = path[i](x_idx,0);
        pose.pose.position.y = path[i](y_idx,0);
        ros_path.poses.push_back(pose);
    }
}

template<int x_idx=0, int y_idx=1, typename EigenM = Eigen::Matrix<double,2,1>>
void fromPath(nav_msgs::Path& ros_path, std::vector<EigenM> path) {
    path.clear();
    for(int i = 0; i < path.size(); ++i) {
        EigenM pt = EigenM::Zeros();
        pt(x_idx, 0) = ros_path.poses[i].pose.position.x;
        pt(y_idx, 0) = ros_path.poses[i].pose.position.y;
        path.push_back(pt);
    }
}


}; /* namespace ros */
}; /* namespace jtraj */

#endif