#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <env_gen_music/colormood.h>
#include <sound_play/sound_play.h>

#include <vector>
#include <iostream>

char category_buffer[40];
ros::Subscriber colormood_sub;
int prev_happiness;
int prev_tempo;
int same_mood;

void pause(int time, ros::NodeHandle &n) {

	if(n.ok()) {
		sleep(time);
	}

}

void colormood_callback(const env_gen_music::colormood::ConstPtr& msg) {
	int numSongs = 4;
	int random = (rand() % 4) + 1;

	/*if(prev_happiness == msg->happiness && prev_tempo == msg->tempo) {
		//don't change songs if we have the same mood
		same_mood = 1;
	} else {*/
		same_mood = 0;
		if (msg->happiness == 1) { 
			if (msg->tempo == 1) { 
				//happy_fast
				sprintf(category_buffer, "music/happy_fast%d.wav", random);	
			} else {
				//happy_slow
				sprintf(category_buffer, "music/happy_slow%d.wav", random);	
			}
		} else {
			if (msg->tempo == 1) { 
				//sad_fast	
				sprintf(category_buffer, "music/sad_fast%d.wav", random);	
			}
			else {
				// sad_slow
				sprintf(category_buffer, "music/sad_slow%d.wav", random);	
			}
		}
	//}
	prev_happiness = msg->happiness;
	prev_tempo = msg->tempo;

	//if (person) {
		// person
	//}
}

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "sound_player");
	sound_play::SoundClient sc;
	ros::NodeHandle n;
	colormood_sub = n.subscribe("/colormood", 1, colormood_callback);
	ros::Rate r(100);

	while (ros::ok()) {
    		ros::spinOnce();
		//printf("same_mood : %d\n", same_mood); //debug
		if(category_buffer[0] != 0/* & same_mood == 0*/) { //only publish if the string isn't empty
			printf("%s\n", category_buffer); //debug
			sc.playWaveFromPkg("env_gen_music", category_buffer);
    			pause(20, n);
		}
		r.sleep();
	}
	return 0;

}
