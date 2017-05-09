#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <env_gen_music/colormood.h>
#include <sound_play/sound_play.h>

#include <vector>
#include <iostream>

char category_buffer[40];
ros::Subscriber colormood_sub;

void colormood_callback(const env_gen_music::colormood::ConstPtr& msg) {
	int numSongs = 4;
	int random = (rand() % 4) + 1;

	if (msg->happiness == 1) { 
		if (msg->tempo == 1) { 
			//happy_fast
			sprintf(category_buffer, "music/happy_fast%d.wav", random);	
		}
		else {
			//happy_slow
			sprintf(category_buffer, "music/happy_slow%d.wav", random);	
		}
	}
	else {
		if (msg->tempo == 1) { 
			//sad_fast	
			sprintf(category_buffer, "music/sad_fast%d.wav", random);	
		}
		else {
			// sad_slow
			sprintf(category_buffer, "music/sad_slow%d.wav", random);	
		}
	}

	//debug statements
//	ROS_INFO("random: %d, string: %s\n", random, category_buffer);

	//if (person) {
		// person
	//}
}

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "sound_player");
	sound_play::SoundClient sc;
	ros::NodeHandle n;
	colormood_sub = n.subscribe("/colormood", 1, colormood_callback);
	ros::Rate r(10000);

	while (ros::ok()) {
    		ros::spinOnce();
		printf("%s\n", category_buffer);
    		sc.playWaveFromPkg("env_gen_music", category_buffer);
    		r.sleep();
		// TODO how do we wait for 20 seconds for song to play?
	}
	return 0;

}
