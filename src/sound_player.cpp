#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <env_gen_music/colormood.h>
#include <sound_play/sound_play.h>
#include <math.h> 
#include <vector>
#include <iostream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

char category_buffer[40];
ros::Subscriber colormood_sub;
ros::Subscriber pcl_perception_sub;
//to track previous state
int prev_happiness = 2;
int prev_tempo = 2;
int same_mood = 0;
int cur_mood = -1;
int new_mood;
int mood_change_counter = 0;
double mood_change_determiner = 0;
int flag = -1;

//to figure out whether to publish new song
int sound_pub = 1;

//to track whether a person is present in frame
int person = 0;
int person_not_seen = 51;

//for person detection
int person_sum = 0;
int person_loop = 0;
int person_detect = 0;
int person_sound = 0;
int prev_person_sound = person_sound;

//to track how much time has elapsed while playing song
std::time_t mood_music_start;
std::time_t person_music_start;
std::time_t now;
int clip_time;
double time_elapsed;

//array of song lengths for each song in database
int sound_lengths[4][4] =	{{13, 12, 16, 16}, 	//happy_fast
							 {12, 22, 19, 21},	//happy_slow
							 {14, 12, 17, 19},	//sad_fast
							 {19, 18, 19, 15}};	//sad_slow

const char* mood_strings[2][2] = {{"sad_slow", "sad_fast"},
						  {"happy_slow", "happy_fast"}};


void colormood_callback(const env_gen_music::colormood::ConstPtr& msg) {
	//random number for song choice
	int numSongs = 4;
	int random = (rand() % 4) + 1;

	//calculates current time
	now = std::time(NULL);
	time_elapsed = std::difftime(now, mood_music_start);

	//tracks if current mood is same as previous mood
	if(!(prev_happiness == msg->happiness && prev_tempo == msg->tempo) && flag == 0) {
		flag = 1;
	}

	//tracks current mood
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

	//mood is same as last so may not change song based on elapsed time
	if(flag == 1) {
		mood_change_counter ++;
		mood_change_determiner += new_mood;

		if(mood_change_counter == 30) {
			mood_change_determiner /= mood_change_counter;
			new_mood = (int) round(mood_change_determiner);
			if(new_mood != cur_mood) { //change song
				same_mood = 0;
				cur_mood = new_mood;
			} else { //don't change song
				same_mood = 1;
			}
			//reset everything regardless
			mood_change_counter = 0;
			mood_change_determiner = 0;
			flag = 0;
		}
	}

	//not the same mood and not a person, so pick a new song
	if(same_mood == 0 || prev_person_sound != person_sound){
		//only runs when program first begins and there is no previous state
		if(flag == -1) {		
			sound_pub = 1;
			cur_mood = new_mood;
			flag = 0;
		}
		
		//publish song
		sound_pub = 1;
		
		//make sure that a person song is not playing before picking a mood song
		if(person_sound == 0) {
			sprintf(category_buffer, "music/%s%d.wav", mood_strings[msg->happiness][msg->tempo], random);	
			clip_time = sound_lengths[cur_mood][random-1];
		} else {
			sprintf(category_buffer, "music/I_Slay.wav");	
			clip_time = 32;
		}
		
		//default to same mood
		same_mood = 1;
	} else {
		//tell sound_play to play same song again and loop if song has finished
		if(time_elapsed >= clip_time) {
			sound_pub = 1;
		} else { 
			//don't publish song if not ready to loop
			sound_pub = 0;
		}
	}
	
	//store data for next time this method is called (track previous state)
	prev_happiness = msg->happiness;
	prev_tempo = msg->tempo;
	prev_person_sound = person_sound;
}

//for person detection: let program know a person has been detected
void pcl_perception_callback(const visualization_msgs::Marker::ConstPtr& msg) {
	person = 1;
}

int main(int argc, char **argv) {
	//seed randomness on system time
	srand(std::time(NULL));
	
	//ROS initialization
	ros::init(argc, argv, "sound_player");
	ros::NodeHandle n;
	ros::NodeHandle n2;
	sound_play::SoundClient sc(n, "robotsound");
	colormood_sub = n.subscribe("/colormood", 1, colormood_callback);
	pcl_perception_sub = n.subscribe("/segbot_pcl_person_detector/marker", 1, pcl_perception_callback);
	ros::Rate r(10);
	mood_music_start = std::time(NULL);

	//continually look for new information from publisher
	while (ros::ok()) {
    	ros::spinOnce();
		//let callback know that a person has been detected if it has	
		if(person == 1) {
			if(person_detect == 0) { //start detecting people
				person_detect = 1;
			}
			if(person_detect == 1) {	
				person_sum++;
			}
			person = 0;
			person_not_seen = 0;
		} else {
			//stop playing person sound if a person has been gone for a long enough time
			if(person_not_seen > 20) {
				person_detect = 0;
				person_sound = 0;
			}
			person_not_seen++;
		}
		//loop person song if person in frame
		if(person_detect == 1) {
			person_loop++;
		}
		if(person_loop == 1) {
			if(person_sum > 0) {
				person_sound = 1;
			}
			person_loop = 0;
			person_sum = 0;
		}
		
		//handle mood music
		if((category_buffer[0] != 0) && sound_pub == 1) { //only publish if the string isn't empty and sound clip needs to be changed or replayed
			mood_music_start = std::time(NULL);
			sc.stopAll();
			sc.startWaveFromPkg("env_gen_music", category_buffer);
		}
		r.sleep();
	}
	return 0;

}
