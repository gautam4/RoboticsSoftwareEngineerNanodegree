#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv)
{
  //Display marker at pickup location and dropoff
  ros::init(argc, argv, "display_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);

  uint32_t shape= visualization_msgs::Marker::CUBE;
  while (ros::ok())
  {
    visualization_msgs::Marker pickupLocMarker;
    pickupLocMarker.header.frame_id="/map";
    pickupLocMarker.header.stamp= ros::Time::now();
    //Unique marker names
    pickupLocMarker.ns= "pickupLocMarker";
    pickupLocMarker.id= 0;

    pickupLocMarker.type= shape;
    
    pickupLocMarker.action = visualization_msgs::Marker::ADD;
    
    pickupLocMarker.pose.position.x = -12.0;
    pickupLocMarker.pose.position.y = 3.0;
    pickupLocMarker.pose.position.z = 0;
    pickupLocMarker.pose.orientation.x = 0.0;
    pickupLocMarker.pose.orientation.y = 0.0;
    pickupLocMarker.pose.orientation.z = 0.0;
    pickupLocMarker.pose.orientation.w = 1.0;

    pickupLocMarker.scale.x = 0.5;
    pickupLocMarker.scale.y = 0.5;
    pickupLocMarker.scale.z = 0.5;

    pickupLocMarker.color.r = 1.0f;
    pickupLocMarker.color.g = 0.0f;
    pickupLocMarker.color.b = 0.0f;
    pickupLocMarker.color.a = 1.0;
    
    pickupLocMarker.lifetime = ros::Duration(5);
    
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    ROS_INFO("Marker Published at Pickup location");
    marker_pub.publish(pickupLocMarker);
    sleep(5);
    //Delete marker
    pickupLocMarker.action = visualization_msgs::Marker::DELETE;
    //pickupLocMarker.lifetime = ros::Duration(5);
    ROS_INFO("Marker Deleted at Pickup location");
    marker_pub.publish(pickupLocMarker);
    
    sleep(5);
     
    //Add marker at DropOff Location
    pickupLocMarker.action = visualization_msgs::Marker::ADD;
    
    pickupLocMarker.pose.position.x = 3.0;
    pickupLocMarker.pose.position.y = -6.0;
    pickupLocMarker.pose.position.z = 0;
    pickupLocMarker.pose.orientation.x = 0.0;
    pickupLocMarker.pose.orientation.y = 0.0;
    pickupLocMarker.pose.orientation.z = 0.0;
    pickupLocMarker.pose.orientation.w = 1.0;

    ROS_INFO("Marker Published at Dropoff Location");
    marker_pub.publish(pickupLocMarker);

    sleep(5);
    
  }
}
