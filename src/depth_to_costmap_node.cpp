/*
  * OpenCV Example using ROS and CPP
  */

 // Include the ROS library
 #include <ros/ros.h>

#include <opencv2/opencv.hpp>
 // Include opencv2
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>

 // Include CvBridge, Image Transport, Image msg
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;


class depth_to_costmap
{
public:
  ros::Subscriber sub_depth, sub_info;
  float fx_inv, fy_inv, fx, fy;
  float cx, cy;
  int height, width;
  bool cam_init;

  float max_height;
  float min_height;
  float height_range_inv;

  float costmap_length_m;
  float costmap_width_m;
  float resolution_m;
  int costmap_height_p, costmap_width_p;
  float cam_pitch;

  int filt_size;

  depth_to_costmap(ros::NodeHandle &nh) // constructor
  {
    cam_init = false;

    sub_depth = nh.subscribe("image",1,&depth_to_costmap::image_cb,this);
    sub_info = nh.subscribe("camera_info",10,&depth_to_costmap::info_cb,this);

    if(not nh.getParam("depth/cam_pitch",cam_pitch))
    {
      cam_pitch = 0;
    }
    if(not nh.getParam("depth/max_height", max_height))
    {
      max_height = 0.5;
    }
    if(not nh.getParam("depth/min_height", min_height))
    {
      min_height = -1.3;
    }
    if(not nh.getParam("depth/costmap_length_m", costmap_length_m))
    {
      costmap_length_m = 20;
    }
    if(not nh.getParam("depth/costmap_width_m", costmap_width_m))
    {
      costmap_width_m = 10;
    }
    if(not nh.getParam("depth/resolution_m", resolution_m))
    {
      resolution_m = 0.1;
    }
    if(not nh.getParam("depth/filt_size", filt_size))
    {
      filt_size = 3;
    }

    ROS_INFO("%f", cam_pitch);
    
    costmap_height_p = costmap_length_m/resolution_m;
    costmap_width_p  = costmap_width_m/resolution_m;
    height_range_inv = max_height - min_height;
  }

  void info_cb(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    fx = info_msg->K[0];
    fy = info_msg->K[4];
    fx_inv = 1.0f/fx;
    fy_inv = 1.0f/fy;
    cx = info_msg->K[2];
    cy = info_msg->K[5];
    height = info_msg->height;
    width = info_msg->width;
    if(!cam_init)
    { 
      cam_init = true;
    }
  }

  void image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    if(!cam_init)
    {
      return;
    }
    
    bool float_flag = false;
    float depth, x, y, z, pixel_height, pixel_val, temp_x, temp_z;
    float ddepth_x, ddepth_y, dzdx, dydx, pixel_grad;
    int row_pix, column_pix;

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat map = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    cv::Mat gradmap = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);

    try
    {

      cv::Mat grad_x, grad_y, smoothened;
      if(msg->encoding == enc::TYPE_16UC1 || msg->encoding == enc::MONO16)
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::GaussianBlur(cv_ptr->image, smoothened, cv::Size(filt_size, filt_size), 0, 0, cv::BORDER_DEFAULT);
        cv::Sobel(smoothened, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
        cv::Sobel(smoothened, grad_y, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
        float_flag = false;
      }
      else if(msg->encoding == enc::TYPE_32FC1)
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::GaussianBlur(cv_ptr->image, smoothened, cv::Size(filt_size, filt_size), 0, 0, cv::BORDER_DEFAULT);
        cv::Sobel(smoothened, grad_x, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
        cv::Sobel(smoothened, grad_y, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
        float_flag = true;
      }

      // u is width, v is height. for opencv "at", row comes first(so height, width or y,x and not x,y)
      for(int u = 0; u < width; u++)
      {
        for(int v = 0; v < height; v++)
        {
          if(float_flag)
          {
            depth = cv_ptr->image.at<float>(v,u);
            ddepth_x = grad_x.at<float>(v,u);
            ddepth_y = grad_y.at<float>(v,u);
          }
          else
          {
            depth = float(cv_ptr->image.at<u_int16_t>(v,u))*1e-3;
            ddepth_x = float(grad_x.at<int16_t>(v,u))*1e-3;
            ddepth_y = float(grad_y.at<int16_t>(v,u))*1e-3;
          }
          
          x = depth;
          z = (cy - float(v))*x*fy_inv;
          y = (cx - float(u))*x*fx_inv;

          temp_x = x;
          temp_z = z;

          x = temp_x*cosf(cam_pitch) - temp_z*sinf(cam_pitch);
          z = temp_x*sinf(cam_pitch) + temp_z*cosf(cam_pitch);

          dzdx = - fy_inv * depth / ddepth_y;
          dydx = - fx_inv * depth / ddepth_x;

          dzdx = tanf(atanf(dzdx) + cam_pitch);

          if(x < costmap_length_m and fabs(y) < costmap_width_m*0.5 and z < max_height)
          {
            row_pix = int(x / resolution_m);
            column_pix = int((y + costmap_width_m*0.5) / resolution_m);
            pixel_grad = gradmap.at<float>(row_pix, column_pix);
            gradmap.at<float>(row_pix, column_pix) = std::min(std::max(pixel_grad, dzdx),1.0f);
          }

          if(z < max_height and z > min_height and x < costmap_length_m and fabs(y) < costmap_width_m*0.5 )
          {
            row_pix = int(x / resolution_m);
            column_pix = int((y + costmap_width_m*0.5) / resolution_m);
            pixel_height = map.at<float>(row_pix, column_pix);

            if(z > 0.05)
            {
              pixel_val = 1.0;
            }
            else
            {
              pixel_val = 0;// 0.1 + 0.9*z*height_range_inv;
            }
            map.at<float>(row_pix, column_pix) = std::max(pixel_val, pixel_height);
          }
          
        }
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    cv::Point p1(1/resolution_m + costmap_width_p/2, 1/resolution_m);
    // Bottom Right Corner
    cv::Point p2(-1/resolution_m + costmap_width_p/2, 0);
  
    // Drawing the Rectangle
    cv::rectangle(map, p1, p2,
              1,
              2, cv::LINE_8);
    cv::Mat display;
    cv::flip(map, display, -1);
    cv::Mat grad_disp;
    cv::flip(gradmap, grad_disp, -1);
    cv::imshow("gradmap", grad_disp);
    cv::imshow("map", display);
    // cv::imshow("test", cv_ptr->image);
    cv::waitKey(3);

  }
};
int main(int argc, char **argv)
{
  //initialize node
  ros::init(argc, argv, "cv_example");
  ros::NodeHandle nh("~");
  depth_to_costmap d2c(nh);
  // subsribe topic
  ros::spin();

  return 0;
}