#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/LandingTarget.h>

#include <string.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_to_mavros");
  ros::NodeHandle node;
  
  /** 
   * Variables 
  **/
  ros::Publisher camera_pose_publisher = node.advertise<geometry_msgs::PoseStamped>("vision_pose", 10);
  ros::Publisher body_path_pubisher = node.advertise<nav_msgs::Path>("body_frame/path", 1);

  tf::TransformListener tf_listener;
  tf::StampedTransform transform;

  geometry_msgs::PoseStamped msg_body_pose;

  nav_msgs::Path body_path;

  std::string target_frame_id = "/camera_odom_frame";
  std::string source_frame_id = "/camera_link";

  double output_rate = 30;
  // 1) r,p,y: align current camera frame with default camera frame (x forward, y left, z up)
  // 2) gamma: align default camera frame's x axis with world y axis
  double roll_cam = 0;
  double pitch_cam = 0;
  double yaw_cam = 0;
  double gamma_world = -1.5707963;
  
  // Wait for the first transform to become available.
  tf_listener.waitForTransform(target_frame_id, source_frame_id, ros::Time::now(), ros::Duration(5.0));

  ros::Time last_tf_time = ros::Time::now();
  ros::Rate rate(output_rate);

  while (node.ok())
  {
    ros::Time now = ros::Time(0);

    try
    {
      tf_listener.lookupTransform(target_frame_id, source_frame_id, now, transform);

      // Only publish pose messages when we have new transform data.
      if (last_tf_time < transform.stamp_)
      {
        last_tf_time = transform.stamp_;

        static tf::Vector3 position_orig, position_body;
        static tf::Quaternion quat_cam, quat_cam_to_body_x, quat_cam_to_body_y, quat_cam_to_body_z, quat_rot_z, quat_body;

        // Rotation from original world frame to world frame with y forward.
        position_orig = transform.getOrigin();

        position_body.setX( cos(gamma_world) * position_orig.getX() + sin(gamma_world) * position_orig.getY());
        position_body.setY(-sin(gamma_world) * position_orig.getX() + cos(gamma_world) * position_orig.getY());
        position_body.setZ(position_orig.getZ());

        // Rotation from camera to body frame.
        quat_cam = transform.getRotation();

        quat_cam_to_body_x = tf::createQuaternionFromRPY(roll_cam, 0, 0);
        quat_cam_to_body_y = tf::createQuaternionFromRPY(0, pitch_cam, 0);
        quat_cam_to_body_z = tf::createQuaternionFromRPY(0, 0, yaw_cam);
        
        // Rotate body frame 90 degree (align body x with world y at launch)
        quat_rot_z = tf::createQuaternionFromRPY(0, 0, -gamma_world);

        quat_body = quat_rot_z * quat_cam * quat_cam_to_body_x * quat_cam_to_body_y * quat_cam_to_body_z;
        quat_body.normalize();

        // Create PoseStamped message to be sent
        msg_body_pose.header.stamp = transform.stamp_;
        msg_body_pose.header.frame_id = transform.frame_id_;
        msg_body_pose.pose.position.x = position_body.getX();
        msg_body_pose.pose.position.y = position_body.getY();
        msg_body_pose.pose.position.z = position_body.getZ();
        msg_body_pose.pose.orientation.x = quat_body.getX();
        msg_body_pose.pose.orientation.y = quat_body.getY();
        msg_body_pose.pose.orientation.z = quat_body.getZ();
        msg_body_pose.pose.orientation.w = quat_body.getW();

        // Publish pose of body frame in world frame
        camera_pose_publisher.publish(msg_body_pose);

        // Publish trajectory path for visualization
        body_path.header.stamp = msg_body_pose.header.stamp;
        body_path.header.frame_id = msg_body_pose.header.frame_id;
        body_path.poses.push_back(msg_body_pose);
        body_path_pubisher.publish(body_path);
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    rate.sleep();
  }
  return 0;
}