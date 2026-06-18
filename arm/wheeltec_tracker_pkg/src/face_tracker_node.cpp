#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    string  face ;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    //bool get_face=nh_.getParam("cascade_1",face);
    cv::Mat image,image_gray,face_result;
    CascadeClassifier face_cascade;
    //获取haar特征的级联表的XML文件，文件路径在launch文件中传入
public:
    
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
      vector<Rect> faceRect;
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
    image = cv_ptr->image;
    cv::cvtColor(image,image_gray,CV_RGBA2GRAY);
    cv::equalizeHist(image_gray,image_gray);
    string path = "data/face.xml";
    if(face_cascade.load(path))
    {
        face_cascade.detectMultiScale(image_gray,faceRect,1.1,3,0,cv::Size(30,30));
        for (size_t i = 0; i < faceRect.size(); i++)
        {
            cv::rectangle(cv_ptr->image,faceRect[i],Scalar(0,0,255),-1);
        }
        

        // Draw an example circle on the video stream
        // if (image_gray.rows > 60 && image_gray.cols > 60)
        //   cv::circle(image_gray, cv::Point(50, 50), 30, CV_RGB(255,0,0),-1);

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

    // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
  }

};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
