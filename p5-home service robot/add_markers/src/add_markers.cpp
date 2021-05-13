//includes
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
//pickup pose as in picking.cpp node (x,y)
double pick_x = -4.0;
double pick_y = 2.0;
//dropping pose (x,y) as in picking.cpp node
double drop_x = 3.0;
double drop_y = 5.0;
//ask if the virtual item picked or not 

bool item_picked_up = false;
bool item_dropped_off = false;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
 // robot pose 
  double robotpose_x = msg->pose.pose.position.x;
  double robotpose_y = msg->pose.pose.position.y;
//relation between the robot pose and the pickin&droping points 
  double pickup_distance = sqrt(pow(robotpose_x - pick_x, 2) + pow(robotpose_y - pick_y, 2));
  double dropoff_distance = sqrt(pow(robotpose_x - drop_x, 2) + pow(robotpose_y - drop_y, 2));

  if (pickup_distance < 0.3) {
    item_picked_up = true;
  }

  if (dropoff_distance < 0.3) {
    item_dropped_off = true;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 10, OdomCallback);

  visualization_msgs::Marker marker;

  // marker shape
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "cube";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pick_x;
  marker.pose.position.y = pick_y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  //  marker scale
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  while (ros::ok()) {
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
// asking if the robot get the picking zone or not if it done then we will delete the marker and stay untill the robot get the drop zone and we will make the marker appears there at the new location
    if (!item_picked_up) {
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
    } else if (!item_dropped_off) {
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
    } else {
      marker.pose.position.x = drop_x;
      marker.pose.position.y = drop_y;
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
    }

    ros::spinOnce();
  }
}