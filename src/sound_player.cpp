#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include <env_gen_music/colormood.h>

#include <vector>
#include <iostream>

//String category;
ros::Subscriber colormood_sub;

// callback function: if hat is within threshold value of middle of window, don't move, otherwise move in correct direction in increments
void colormood_callback(const env_gen_music::colormood::ConstPtr& msg) {
    int numSongs = 4;
	// get random number based on numSongs
	int rand;

	if (msg->happiness) {
		if (msg->tempo) {
			// happy_fast
			// string = "happy_fast%d.wav", rand
		}
		else {
			// happy_slow
		}
	}
	else {
		if (msg->tempo) {
			// slow_fast
		}
		else {
			// sad_slow
		}
	}

	if (person) {
		// person
	}
}

int main(int argc, char **argv)
{
  sound_play::SoundClient sc;
  ros::init(argc, argv, "move_base_client");
  ros::NodeHandle n;
  colormood_sub = n.subscribe("/colormood", 1, colormood_callback);
  ros::Rate r(100);

//  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
 
//  move_base_msgs::MoveBaseGoal goal;
  
  while (ros::ok()) {
    ros::spinOnce();
	
//int main(int argc, char ** argv) {
// TODO do we need these 2 lines?
//    ros::init(argc, argv, "example");
//    ros::NodeHandle nh;

    sc.playWaveFromPkg("sound_play", /*category*/"sounds/BACKINGUP.ogg");

//   ac.sendGoal(goal);
    r.sleep();
    //block until the action is completed
	// TODO how do we wait for 20 seconds for song to play?
 //   ac.waitForResult(ros::Duration(0.1));
  }
  return 0;

}
