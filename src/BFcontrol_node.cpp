#include "BFcontrol_node.h"
  
  
  
  
double Odom::LinearControl::quaternion2yaw(Eigen::Quaterniond q){
    double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
    return yaw;
}