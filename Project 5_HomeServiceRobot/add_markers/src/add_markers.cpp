#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>


//Define a PubSub Class

class addMarkerPubSub
{
public: 
  addMarkerPubSub()
  {
    //Define publisher and subscriber
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);
    marker_sub = n.subscribe("/odom", 10, &addMarkerPubSub::markerCallback, this);
    //Phase is the different stages of the delivery process
    homeServicePhase = 0;
  }
  
  void markerCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    //Extract position x and y from odom message
    robotLoc_X = msg->pose.pose.position.x;
    robotLoc_Y = msg->pose.pose.position.y;
    
    //Define marker at pickup location
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker pickupLocMarker;
    pickupLocMarker.header.frame_id="/map";
    pickupLocMarker.header.stamp= ros::Time::now();
  
    pickupLocMarker.ns= "pickupLocMarker";
    pickupLocMarker.id= 0;

    pickupLocMarker.type= shape;
    pickupLocMarker.action= visualization_msgs::Marker::ADD;
    
    if ((homeServicePhase == 0) && ((robotLoc_X >= -11.0) && (robotLoc_Y <= 3.5)))
    {
      //Robot started on its way to pickup location
      // Show marker at pickup location
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
  
      pickupLocMarker.lifetime = ros::Duration();
  
      ROS_INFO("Robot is on the way to pick up the object");
      marker_pub.publish(pickupLocMarker); 
      
    }
    else if ((homeServicePhase == 0) && ((robotLoc_X <= -11.0) && (robotLoc_Y >= 3.5)))
    {
      //Robot is near pickup radius
      ROS_INFO("Robot is within the pickup radius");
      homeServicePhase = 1;
      
    }
    else if ((homeServicePhase == 1) && ((robotLoc_X <= -11.0) && (robotLoc_Y >= 3.5)))
    {
      //Robot is in pickup radius
      //Marker disappears, visually simulating the robot "picking up" the object
      ROS_INFO("Robot is picking up object");
      pickupLocMarker.action= visualization_msgs::Marker::DELETE;
      marker_pub.publish(pickupLocMarker); 
      ROS_INFO("Robot has picked up object");
      homeServicePhase = 2;
      
    }
    else if ((homeServicePhase == 2) && ((robotLoc_X >= -11.0) && (robotLoc_Y <= 3.5)))
    {
      ROS_INFO("Robot is heading to the dropoff location");
      //Used to test the conditional statements 
      //ROS_INFO("Robot X Loc %f", robotLoc_X);
      //ROS_INFO("Robot Y Loc %f", robotLoc_Y);
     
      if ((robotLoc_X >= 2.3) && (robotLoc_Y <= -4.0))
      {
        homeServicePhase = 3;
      }
      
    }
    else if ((homeServicePhase == 3) && ((robotLoc_X >= 2.4) && (robotLoc_Y <= -4.0)))
    {
      ROS_INFO("Robot is within dropoff location");
      //Further distance narrowing before displaying the marker
      if ((robotLoc_X >= 2.5) && (robotLoc_Y <= -4.0))
      {
        homeServicePhase = 4;
      }
      
    }
    else if ((homeServicePhase == 4) && ((robotLoc_X >= 2.6) && (robotLoc_Y <= -4.0)))
    {
      ROS_INFO("Robot is delivering the object");
      //Define a new marker for delivery location 
      visualization_msgs::Marker deliveryLocMarker;
      //Used to test the conditional statements
      //ROS_INFO("Robot X Loc %f", robotLoc_X);
      //ROS_INFO("Robot Y Loc %f", robotLoc_Y);

      deliveryLocMarker.header.frame_id="/map";
      deliveryLocMarker.header.stamp= ros::Time::now();
  
      deliveryLocMarker.ns= "deliveryLocMarker";
      deliveryLocMarker.id= 0;

      deliveryLocMarker.type= shape;
      deliveryLocMarker.action= visualization_msgs::Marker::ADD;
      
      deliveryLocMarker.pose.position.x = 3.5;
      deliveryLocMarker.pose.position.y = -6.0;
      deliveryLocMarker.pose.position.z = 0;
      deliveryLocMarker.pose.orientation.x = 0.0;
      deliveryLocMarker.pose.orientation.y = 0.0;
      deliveryLocMarker.pose.orientation.z = 0.0;
      deliveryLocMarker.pose.orientation.w = 1.0;

      deliveryLocMarker.scale.x = 0.5;
      deliveryLocMarker.scale.y = 0.5;
      deliveryLocMarker.scale.z = 0.5;

      deliveryLocMarker.color.r = 1.0f;
      deliveryLocMarker.color.g = 0.0f;
      deliveryLocMarker.color.b = 0.0f;
      deliveryLocMarker.color.a = 1.0;
  
      pickupLocMarker.lifetime = ros::Duration();

      marker_pub.publish(deliveryLocMarker); 
      ROS_INFO("Robot has delivered the object");
      sleep(2);

    }

  }

private:
  ros::NodeHandle  n;
  ros::Publisher marker_pub;
  ros::Subscriber marker_sub;
  double robotLoc_X;
  double robotLoc_Y;
  int homeServicePhase;

};


int main( int argc, char** argv)
{
  //Initialize the node
  ros::init(argc, argv, "add_markers");
  //Define class instance
  addMarkerPubSub markerAdd;

  ros::spin();
  return 0;
}
