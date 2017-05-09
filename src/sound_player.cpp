#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <env_gen_music/colormood.h>
#include <sound_play/sound_play.h>
#include <math.h> 
#include <vector>
#include <iostream>

char category_buffer[40];
ros::Subscriber colormood_sub;
int prev_happiness = 2;
int prev_tempo = 2;
int same_mood = 0;
int sound_pub = 1;
int cur_mood = -1;
int new_mood;
int mood_change_counter = 0;
double mood_change_determiner = 0;
int flag = -1;

std::time_t start;
std::time_t now;
int clip_time;
double time_elapsed;

int sound_lengths[4][4] =	{{13, 12, 16, 16}, 	//happy_fast
							 {12, 22, 19, 21},	//happy_slow
							 {14, 12, 17, 19},	//sad_fast
							 {19, 18, 19, 15}};	//sad_slow

const char* mood_strings[2][2] = {{"sad_slow", "sad_fast"},
						  {"happy_slow", "happy_fast"}};


void pause(int time, ros::NodeHandle &n) {

	if(n.ok()) {
		sleep(time);
	}

}

void colormood_callback(const env_gen_music::colormood::ConstPtr& msg) {
	int numSongs = 4;
	int random = (rand() % 4) + 1;

	now = std::time(NULL);
	time_elapsed = std::difftime(now, start);

	if(!(prev_happiness == msg->happiness && prev_tempo == msg->tempo) && flag == 0) {
		flag = 1;
		printf("detected slight mood change\n");
	}

//	printf("time elapsed: %f\n", time_elapsed);
	if (msg->happiness == 1) { 
		if (msg->tempo == 1) { 
			//happy_fast
			new_mood = 0;
		} else {
			//happy_slow
			new_mood = 1;
		}
	} else {
		if (msg->tempo == 1) { 
			//sad_fast	
			new_mood = 2;
		}
		else {
			// sad_slow
			new_mood = 3;
		}
	}

	if(flag == 1) {
//		printf("flag = 1\n");
		mood_change_counter ++;
		mood_change_determiner += new_mood;
		if(mood_change_counter == 10000) {
			printf("may or may not be changing mood\n");
			mood_change_determiner /= mood_change_counter;
			new_mood = (int) round(mood_change_determiner);
			if(new_mood != cur_mood) {
				sound_pub = 1;
				same_mood = 0;
				cur_mood = new_mood;
			} else {
				same_mood = 1;
			}
			mood_change_counter = 0;
			mood_change_determiner = 0;
			flag = 0;
		}
	}
	if(same_mood == 0){
		if(flag == -1) {
			printf("flag = -1\n");			
			sound_pub = 1;
			cur_mood = new_mood;
			flag = 0;
		}
		if(time_elapsed >= clip_time) {
			sound_pub = 1;
//				printf("sound clip time: %d\n", sound_lengths[3][random-1]);
		} else {
			sound_pub = 0;
		}
	}
	if(sound_pub == 1) {
		sprintf(category_buffer, "music/%s%d.wav", mood_strings[msg->happiness][msg->tempo], random);	
		clip_time = sound_lengths[cur_mood][random-1];
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
//		printf("time elapsed: %f\n", time_elapsed);
		if((category_buffer[0] != 0) && sound_pub == 1) { //only publish if the string isn't empty and sound clip needs to be changed or replayed
			start = std::time(NULL);
			printf("time elapsed: %f\n", time_elapsed);
			printf("in main: %s\n", category_buffer); //debug
			sc.playWaveFromPkg("env_gen_music", category_buffer);
//    			pause(clip_time, n);
		}
		r.sleep();
	}
	return 0;

}
