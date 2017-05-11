#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher marker_pub;

int main(int argc, char** argv) {

	ros::init(argc, argv, "test_marker_pub");
	ros::NodeHandle n;	
	marker_pub = n.advertise<visualization_msgs::Marker>("segbot_pcl_person_detector/marker", 1000);
  	ros::Rate r(10);

	int on = 0;

	while (ros::ok()) {
    	ros::spinOnce();
		if(on == 50) {
			on = 0;
			printf("publishing marker\n");
			visualization_msgs::Marker marker;
			marker_pub.publish(marker);
		} else {
			on++;
		}
		r.sleep();
	}
	return 0;
}
