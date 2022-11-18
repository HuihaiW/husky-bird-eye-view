#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;

int main(int argc, char** argv){

  Matrix3d Homography, intrinsic_rotate, rotation_matrix, temp_matrix;
   Homography << 0.00087138, -0.000819049, 0.716003, 7.13997e-05, -0.000203317, 0.698095, 9.45712e-08, -1.27232e-06, 0.00112625;

  intrinsic_rotate << 606.782, 0.0,  643.805,
                        0.0, 606.896,  366.084,
                        0.0, 0.0,  1.0;

  temp_matrix = intrinsic_rotate.inverse() * Homography;
  Matrix<double, 3, 1> R1, R2, R3, translation;
  R1 << temp_matrix(0, 0), temp_matrix(1, 0), temp_matrix(2,0);
  R2 << temp_matrix(0, 1), temp_matrix(1, 1), temp_matrix(2,1);
  R3 = R1.normalized().cross(R2.normalized());
  rotation_matrix << R1.normalized(), R2.normalized(), R3;

  Quaterniond q(rotation_matrix);

  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle node;

  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;

  
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "rgb_camera_link";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 2.0;
  transformStamped.transform.translation.z = 0.0;

  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  ros::Rate rate(30.0);
  while (node.ok()){
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);
    rate.sleep();
    printf("sending\n");
  }

};
