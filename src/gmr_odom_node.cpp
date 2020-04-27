#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

double wl, wr;

void leftCb(const std_msgs::Float32::ConstPtr &msg)
{
  wl = msg->data;
}
void rightCb(const std_msgs::Float32::ConstPtr &msg)
{
  wr = msg->data;
}

int main(int argc, char **argv)
{
  // Start ROS within the context of this node.
  ros::init(argc, argv, "gmr_odom_node");
  // Declare node.
  ros::NodeHandle nh;
  ros::Rate loop_rate(50);      //Hz

  ros::Subscriber sub_left_rpm = nh.subscribe ("/left_rpm",1, leftCb);
  ros::Subscriber sub_right_rpm = nh.subscribe("/right_rpm",1, rightCb);
  ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 1);

  double axle_track, wheel_radius, gear_ratio, v, w, vl, vr;
  nh.param("axle_track", axle_track, 1.0);
  nh.param("gear_ratio", gear_ratio, 1.0);
  nh.param("wheel_radius", wheel_radius, 1.0);

  nav_msgs::Odometry odom;
  odom.header.frame_id = "/map";
  odom.child_frame_id = "/base_link";
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = "/map";
  odom_trans.child_frame_id = "/base_link";

  double x = 0.0, y = 0.0, th = 0.0, dt;
  
  tf::TransformBroadcaster  broadcaster;

  ros::Time cur_t, last_t = ros::Time::now();

  while(ros::ok())
  {
    cur_t = ros::Time::now();
    vl = wl/gear_ratio*wheel_radius*2*M_PI/60.0;
    vr = wr/gear_ratio*wheel_radius*2*M_PI/60.0;
    v = (vr+vl)/2;
    w = (vr-vl)/axle_track;

    dt = cur_t.toSec() - last_t.toSec();
    last_t = cur_t;
    th += w*dt;
    x += v*std::cos(th)*dt;
    y += v*std::sin(th)*dt;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    odom.header.stamp = cur_t;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = odom_quat;


    //first, we'll publish the transform over tf
    odom_trans.header.stamp = cur_t;
    odom_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.1;
    odom_trans.transform.rotation = odom_quat;
    broadcaster.sendTransform(odom_trans);

    pub_odom.publish(odom);
    ros::spinOnce();
    loop_rate.sleep();
  }

}
