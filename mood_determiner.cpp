#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>  
//TODO great message named colormood
#include <packagename/colormood.h>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OUT_WINDOW = "Output window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher colormood_pub;
  int counter;
  int x;
  int y;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/nav_kinect/rgb/image_color", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    // publish to "hat_pos" 
    colormood_pub = nh_.advertise<packagename::colormood>("colormood", 1);
    cv::namedWindow(OPENCV_WINDOW);
    r = 0;
    g = 0;
    b = 0;
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    
    //declare output image
    cv::Mat outImg;
    r = 0;
    g = 0;
    b = 0;
    packagename::moodcolor mood_msg;

	outImg = cv_ptr->image.clone(); 
	for (unsigned int i = 0; i < outImg.rows; i ++){
		for (unsigned int j = 0; j < outImg.cols; j ++){
			//example of how to look up pixel values
			int b_ij = (int)outImg.at<cv::Vec3b>(i,j)[0];
			int g_ij = (int)outImg.at<cv::Vec3b>(i,j)[1];
			int r_ij = (int)outImg.at<cv::Vec3b>(i,j)[2];
			r += r_ij;
			g += g_ij;
			b += b_ij;
		}
	}

	int mood;
	// TODO determine mood here with r g b

	mood_msg.mood = mood;
	colormood_pub.publish(mood);
	

	//show input
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    
	//show output
	cv::imshow(OUT_WINDOW, outImg);
    
	//pause for 3 ms
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
