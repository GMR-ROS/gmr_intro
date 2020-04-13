#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iterator>
#include <random>
#include <std_srvs/Trigger.h>

bool toogle_robot = true;

bool toogleRobot(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  toogle_robot = !toogle_robot;

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
  ros::ServiceServer service = nh.advertiseService("/toogle_robot", toogleRobot);

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

  ros::Time current_time, first_time = ros::Time::now(); 
  ros::Duration(0.02).sleep();

  while(ros::ok())
  {
    nh.setParam("/axle_track", axle_track);
    nh.setParam("/gear_ratio", gear_ratio);
    nh.setParam("/wheel_radius", wheel_radius);
    current_time = ros::Time::now();
    if(int(1000*current_time.toSec()-1000*first_time.toSec())%5000 < 20)
    {     
      toggle_curve = !toggle_curve;
      ROS_INFO_STREAM("Toggle");
    }

    if(toogle_robot)
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
    ros::spinOnce();
    loop_rate.sleep();
  }

}
