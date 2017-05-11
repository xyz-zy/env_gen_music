# Environment-based-Music-Generation
Freshman Research Initiative (FRI) Spring 2017 Final Project

This project adds two nodes to the robot that enable the robot to play music based on its environment.

**Currently, the ROS Indigo sound\_play package is buggy. As per the documentation, the package should allow multiple sounds to play at once, but this is not the case. You can view the open issue in the sound\_play github repository here:
	https://github.com/ros-drivers/audio_common/issues/23
For this project's sound_player node to work correctly, please perform the following, as per the issue's suggested fix:
1. Open the soundplay\_node.py file, in the installed sound_play package directory, as root. This should be located in /opt/ros/indigo/lib/sound\_play/.
2. Comment out line "self.stopAll()" directly under the comment "# Force only one sound at a time". This should be line 261.
3. Save and close. Now sound_player should run correctly.

How to run the nodes:
1. In one terminal window, start roscore.
2. In the second terminal window, either start the robot, or if running in simulation, play a rosbag.
3. In the third terminal window, run the mood\_determiner node from the env\_gen\_music package.
4. In a fourth terminal window, run soundplay\_node.py node from the sound\_play package.
5. In a fifth terminal window, either "roslaunch pcl\_perception background\_people\_detection\_v2.launch", if on the robot, or run the test\_marker\_pub node from the env\_gen\_music package.
6. In the sixth terminal window, run the sound\_player node from the env\_gen\_music package.

In summary, these are the lines you should type into the terminal:

	roscore
	roslaunch segbot_v2.launch OR rosbag play -l <name of rosbag>
	rosrun env_gen_music mood_determiner
	rosrun sound_play soundplay_node.py
	rosrun env_gen_music sound_player

mood\_determiner:
This node takes in the visual data from the camera feed and processes the image to determine
whether the image is predominantly warm or cool colors. Specifically, the node converts the image
to the HSV color space, then finds the average hue and saturation over the image. Based on the hue,
the node outputs a 1 to signal a warm image or a 0 to signal a cold image. The node also outputs
a 1 to signal low saturation and a 0 to signal high saturation.

	Inputs: visual data from camera
	Outputs: warmth of image (1 or 0) and average saturation (1 or 0)
	Subscribers: sound_player

sound\_player:
This node subscribes the data published by the mood\_determiner and uses that to pick a song to play.
The warmth of the image is used to determine the mood of the song, happy for warm images and sad for
cold ones. The saturation of the image is used to determine the tempo of the image, fast for high
saturation images and slow for low saturation images. This means that there are four categories of songs:
happy-fast, happy-slow, sad-fast, and sad-slow. This node picks from a database of songs stored in the
package based on the correct category and then plays that song continuously until the mood changes
for a significant amount of time.

	Inputs: warmth of image and average saturation
	Outputs: file path to a song to play
	Subscribers: soundplay_node.py


Music from: (see music/MusicList.txt for individual clip sources)
PlayOnLoop.com - Licensed under Creative Commons By Attribution 4.0
OrangeFreeSounds.com - Licensed under Creative Commons By Attribution 4.0
