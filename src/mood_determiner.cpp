#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>  
#include <env_gen_music/colormood.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter {
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Publisher colormood_pub;
	int rgb_counter;
	int saturation;
	int hue;
	int sat_counter;
  
	public: ImageConverter() : it_(nh_) {
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/nav_kinect/rgb/image_color", 1, &ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/mood_determiner/output_video", 1);
	
		// publish to "colormood" 
		colormood_pub = nh_.advertise<env_gen_music::colormood>("colormood", 1);
		cv::namedWindow(OPENCV_WINDOW);
		hue = 0;
		saturation = 0;
	}

	~ImageConverter() {
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
		  	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	   	} catch (cv_bridge::Exception& e) {
		  	ROS_ERROR("cv_bridge exception: %s", e.what());
		  	return;
		}
		
		//declare output image
		cv::Mat HSVImg;
		env_gen_music::colormood mood_msg;
	
		//get the average saturation of the image
		saturation = 0;
		hue = 0;
		sat_counter = 0;
		cvtColor(cv_ptr->image.clone(), HSVImg, CV_BGR2HSV);
		for(unsigned int i = 0; i < HSVImg.rows; i++) { //iterate over each pixel in image
			for(unsigned int j = 0; j < HSVImg.cols; j++) {
				int sat_ij = (int)HSVImg.at<cv::Vec3b>(i,j)[1]; //saturation
				int hue_ij = (int)HSVImg.at<cv::Vec3b>(i,j)[0];	//hue
				saturation += sat_ij;
				hue += hue_ij;
				sat_counter++;
			}	
		}
		saturation = saturation / sat_counter;
		hue = hue / sat_counter;

		//subscriber expects 1 for happy/fast and anything else otherwise
		int happiness = (hue > 90 || hue < 60) ? 1 : 0; //warm between 0-60 and 90-180
		int tempo = saturation > 100 ? 1 : 0; //saturation max is 255, saturated between 100-255
	
		//debugging information
		ROS_INFO("saturation: %d, hue: %d, happiness: %d, tempo: %d\n", saturation, hue, happiness, tempo);

		//construct message
		mood_msg.happiness = happiness;
		mood_msg.tempo = tempo;
		colormood_pub.publish(mood_msg);
	

		//show input
	 	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		
		//pause for 3 ms
		cv::waitKey(3);
		
		//Output modified video stream
		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "mood_determiner");
	ImageConverter ic;
	ros::spin();
 	return 0;
}
