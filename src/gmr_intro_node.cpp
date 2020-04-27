#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iterator>
#include <random>
#include <std_srvs/Trigger.h>

bool toggle_robot = true;

bool toggleRobot(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  toggle_robot = !toggle_robot;
  //ROS_WARN_STREAM("toggling robot");
  response.success = true;
  response.message = "Someone toggled me...";
  return true;
}

int main(int argc, char **argv)
{
  // Start ROS within the context of this node.
  ros::init(argc, argv, "gmr_intro_node");
  // Declare node.
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(50);      //Hz

  ros::Publisher pub_left_rpm = nh.advertise<std_msgs::Float32> ("/left_rpm",1);
  ros::Publisher pub_right_rpm = nh.advertise<std_msgs::Float32> ("/right_rpm",1);
  ros::ServiceServer service = nh.advertiseService("/toggle_robot", toggleRobot);

  double axle_track, wheel_radius, gear_ratio, rpm_ref, gaussian_noise_mean, gaussian_noise_stddev;
  nh.param("/axle_track", axle_track, 0.5);                         //meters
  nh.param("/gaussian_noise_mean", gaussian_noise_mean, 0.0);       //rpm
  nh.param("/gaussian_noise_stddev", gaussian_noise_stddev, 1.0);   //rpm
  nh.param("/gear_ratio", gear_ratio, 20.0);                        //ratio
  nh.param("/rpm_ref", rpm_ref, 15.0);                              //rpm
  nh.param("/wheel_radius", wheel_radius, 0.05);                    //meters

  std_msgs::Float32 wl, wr;
  bool toggle_curve = false;

  //ros::ServiceClient client = nh.serviceClient<diagnostics_msgs::AddDiagnostics>("/toggle_the_robot");
  

  // Define random generator with Gaussian distribution
  std::default_random_engine generator_l, generator_r;
  std::normal_distribution<double> dist(gaussian_noise_mean, gaussian_noise_stddev);

  ros::Time current_time, first_time = ros::Time::now(), initial_curve_time = first_time, initial_straight_time = first_time; 
  ros::Duration(0.02).sleep();

  double vl, vr, wz, t_curve;
  vl = -rpm_ref*wheel_radius*2*M_PI/60.0;
  vr = -vl;
  wz = (vr-vl)/axle_track;
  t_curve = 0.5*M_PI/wz;

  while(ros::ok())
  {
    nh.setParam("/axle_track", axle_track);
    nh.setParam("/gear_ratio", gear_ratio);
    nh.setParam("/wheel_radius", wheel_radius);
    current_time = ros::Time::now();
    if(current_time.toSec() - initial_curve_time.toSec() > t_curve && toggle_curve)
    {     
      toggle_curve = !toggle_curve;
      initial_straight_time = current_time;
      ROS_INFO_STREAM("Toggle");
    }
    else if(current_time.toSec() - initial_straight_time.toSec() > 5.0 && !toggle_curve )
    {
      toggle_curve = !toggle_curve;
      initial_curve_time = current_time;
      ROS_INFO_STREAM("Toggle");

    }

    if(toggle_robot)
    {
      if(toggle_curve)
      {
        wl.data = - (rpm_ref + dist(generator_l)) * gear_ratio ;
        wr.data = (rpm_ref + dist(generator_r)) * gear_ratio;
      }
      else
      {
        wl.data = (rpm_ref/4 + dist(generator_l)) * gear_ratio;
        wr.data = (rpm_ref/4 + dist(generator_r)) * gear_ratio;
      }
    }
    else
    {
      wl.data = 0;
      wr.data = 0;
      first_time = current_time;
    }

    pub_left_rpm.publish(wl);
    pub_right_rpm.publish(wr);
    //ROS_INFO_STREAM("speeds published");
    ros::spinOnce();
    loop_rate.sleep();
  }

}
