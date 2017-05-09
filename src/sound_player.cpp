#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <env_gen_music/colormood.h>
#include <sound_play/sound_play.h>

#include <vector>
#include <iostream>

char category_buffer[40];
ros::Subscriber colormood_sub;
int prev_happiness = 2;
int prev_tempo = 2;
int same_mood = 0;

std::time_t start;
std::time_t now;
int clip_time;
double time_elapsed;

int sound_lengths[4][4] =	{{13, 12, 16, 16}, 	//happy_fast
							 {12, 22, 19, 21},	//happy_slow
							 {14, 12, 17, 19},	//sad_fast
							 {19, 18, 19, 15}};	//sad_slow


void pause(int time, ros::NodeHandle &n) {

	if(n.ok()) {
		sleep(time);
	}

}

void colormood_callback(const env_gen_music::colormood::ConstPtr& msg) {
	int numSongs = 4;
	int random = (rand() % 4) + 1;

//	if(!(prev_happiness == msg->happiness && prev_tempo == msg->tempo)) {
	if(time_elapsed >= clip_time) {
		printf("time elapsed: %f\n", time_elapsed);
		start = std::time(NULL);
		same_mood = 0;
		if (msg->happiness == 1) { 
			if (msg->tempo == 1) { 
				//happy_fast
				sprintf(category_buffer, "music/happy_fast%d.wav", random);				
				clip_time = sound_lengths[0][random-1];
			} else {
				//happy_slow
				sprintf(category_buffer, "music/happy_slow%d.wav", random);
				clip_time = sound_lengths[1][random-1];
			}
		} else {
			if (msg->tempo == 1) { 
				//sad_fast	
				sprintf(category_buffer, "music/sad_fast%d.wav", random);					clip_time = sound_lengths[2][random-1];
			}
			else {
				// sad_slow
				sprintf(category_buffer, "music/sad_slow%d.wav", random);					clip_time = sound_lengths[3][random-1];
//				printf("sound clip time: %d\n", sound_lengths[3][random-1]);
			}
		}
	} else {
		same_mood = 1;
	}
	prev_happiness = msg->happiness;
	prev_tempo = msg->tempo;

	//if (person) {
		// person
	//}
}

int main(int argc, char **argv) {
	srand(std::time(NULL));
	ros::init(argc, argv, "sound_player");
	sound_play::SoundClient sc;
	ros::NodeHandle n;
	colormood_sub = n.subscribe("/colormood", 1, colormood_callback);
	ros::Rate r(10);
	start = std::time(NULL);

	while (ros::ok()) {
    		ros::spinOnce();
		//printf("same_mood : %d\n", same_mood); //debug
		now = std::time(NULL);
		time_elapsed = std::difftime(now, start);
//		printf("time elapsed: %f\n", time_elapsed);
		if((category_buffer[0] != 0) && (same_mood == 0)) { //only publish if the string isn't empty
//			printf("in main: %s\n", category_buffer); //debug
			sc.playWaveFromPkg("env_gen_music", category_buffer);
//    			pause(clip_time, n);
		}
		r.sleep();
	}
	return 0;

}
