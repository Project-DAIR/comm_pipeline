#include <ros/ros.h>
#include <time.h>

#include <geometry_msgs/Point.h>


// main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_node");
    ros::NodeHandle nh;
    geometry_msgs::Point marker_pose;

    ros::Publisher marker_pub = nh.advertise<geometry_msgs::Point>("/marker", 10);
    marker_pose.x = 5.0;
    marker_pose.y = 0.0;
    marker_pose.z = 0.0;

    while(ros::ok()){
        ROS_INFO("Marker Pose x: %f y: %f z %f", marker_pose.x, marker_pose.y, marker_pose.z);

        marker_pub.publish(marker_pose);

      ros::spinOnce();
      ros::Duration(10).sleep();
    }
  
  return 0;
}