/*
  * OpenCV Example using ROS and CPP
  */

 // Include the ROS library
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

class depth_to_costmap
{
public:
  ros::Subscriber sub_depth, sub_info, sub_pose, sub_bound;
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
  float pose[3], last_pose[3];
  bool pose_init;

  std::vector<cv::Point2f> bound;
  bool bound_received = false;

  cv::Mat map, gradmap, bound_map;

  depth_to_costmap(ros::NodeHandle &nh) // constructor
  {
    cam_init = false;
    pose_init = false;
    sub_depth = nh.subscribe("image",1,&depth_to_costmap::image_cb,this);
    sub_info = nh.subscribe("camera_info",10,&depth_to_costmap::info_cb,this);
    sub_pose = nh.subscribe("pose",1,&depth_to_costmap::pose_cb, this);
    sub_bound = nh.subscribe("bound",1, &depth_to_costmap::bound_cb, this);

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

    map = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    bound_map = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);
    gradmap = cv::Mat::zeros(cv::Size(costmap_width_p, costmap_height_p),CV_32FC1);

  }

  void rpy_from_quat(float rpy[3],const geometry_msgs::PoseStamped::ConstPtr& msg) 
  { 
    float q[4];
    q[0] = msg->pose.orientation.x;
    q[1] = msg->pose.orientation.y;
    q[2] = msg->pose.orientation.z;
    q[3] = msg->pose.orientation.w;
    rpy[2] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),1 - 2 * (q[1] * q[1] + q[2] * q[2]));
    rpy[0] = asinf(2.0f * (q[0] * q[2] - q[3] * q[1]));
    rpy[1] = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
  }

  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    float rpy[3];
    pose[0] = msg->pose.position.x;
    pose[1] = msg->pose.position.y;
    rpy_from_quat(rpy,msg);
    pose[2] = rpy[2];
    if(not pose_init)
    {
      pose_init = true;
      last_pose[0] = pose[0];
      last_pose[1] = pose[1];
      last_pose[2] = pose[2];
    }

  }

  void bound_cb(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
    cv::Point2f temp;  // temp to hold values
    bound.clear(); // remove last points
    int num = msg->poses.size();  // how many points?
    for(int i = 0; i < num; i++)
    {
      temp.x = msg->poses[i].position.x;
      temp.y = msg->poses[i].position.y;
      bound.push_back(temp);
    }
    bound_received = true;
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
    if(!cam_init or !pose_init)
    {
      return;
    }
    ros::Time begin = ros::Time::now();
    bool float_flag = false;
    float depth, x, y, z, temp_x, temp_z;
    int row_pix, column_pix;

    cv_bridge::CvImagePtr cv_ptr;

    // find delta pose in odom frame
    float delta_x = pose[0] - last_pose[0];
    float delta_y = pose[1] - last_pose[1];
    float delta_t = pose[2] - last_pose[2];
    float delta[3], ct = cosf(-pose[2]), st = sinf(-pose[2]);  // backward rotation
    //  find delta pose in body frame
    delta[0] = delta_x*ct - delta_y*st;
    delta[1] = delta_x*st + delta_y*ct;
    delta[2] = delta_t;

    //  set the last known pose
    last_pose[0] = pose[0];
    last_pose[1] = pose[1];
    last_pose[2] = pose[2];
    
    //  define rotation center as camera's location in map image
    cv::Point2f center(costmap_width_p*0.5, 0);
    cv::Mat rotated_image, translated_image;  // temporary images
    float warp_values[] = { 1.0, 0.0, -delta[1]/resolution_m, 0.0, 1.0, -delta[0]/resolution_m };  // translation matrix
    cv::Mat translation_matrix = cv::Mat(2, 3, CV_32F, warp_values);    // fill in the values
    cv::Mat rotation_matix = getRotationMatrix2D(center, -delta[2]*57.3, 1.0);  // rotate matrix around camera. angle in degrees    
    //  in forward state propagation, we rotate, then translate. Therefore, in backward transform, we translate, then rotate
    cv::warpAffine(map, translated_image, translation_matrix, map.size());  // then we translate
    cv::warpAffine(translated_image, rotated_image, rotation_matix, map.size());  // first we rotate
    map = rotated_image.clone();

    // this is for the bound layer.
    cv::warpAffine(bound_map, translated_image, translation_matrix, map.size());
    cv::warpAffine(translated_image, rotated_image, rotation_matix, map.size());
    bound_map = rotated_image.clone();

    try
    {

      cv::Mat grad_x, grad_y, smoothened;
      if(msg->encoding == enc::TYPE_16UC1 || msg->encoding == enc::MONO16)
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        float_flag = false;
      }
      else if(msg->encoding == enc::TYPE_32FC1)
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        float_flag = true;
      }
      // u is width, v is height. for opencv "at", row comes first(so height, width or y,x and not x,y)
      for(int u = 0; u < width; u++)
      {
        for(int v = height*0.4; v < height*0.8; v++)
        {
          if(float_flag)
          {
            depth = cv_ptr->image.at<float>(v,u);
          }
          else
          {
            depth = float(cv_ptr->image.at<u_int16_t>(v,u))*1e-3;
          }
          
          x = depth;
          z = (cy - float(v))*x*fy_inv;
          y = (cx - float(u))*x*fx_inv;

          temp_x = x;
          temp_z = z;

          x = temp_x*cosf(cam_pitch) - temp_z*sinf(cam_pitch);
          z = temp_x*sinf(cam_pitch) + temp_z*cosf(cam_pitch);

          if(z < max_height and z > min_height and x < costmap_length_m and fabs(y) < costmap_width_m*0.5 )
          {
            row_pix = int(x / resolution_m);
            column_pix = int((y + costmap_width_m*0.5) / resolution_m);
            map.at<float>(row_pix, column_pix) = (z - min_height)*height_range_inv;  // normalize using height range.
          } 
        }
      }
      cv::Sobel(map, gradmap, CV_32F, 1, 1, 3, 1, 0, cv::BORDER_DEFAULT);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    if(bound_received)
    {
      bound_received = false;
      cv::Point2f dum, temp;
      for(int i = 0; i < bound.size(); i++)
      {
        dum.x = bound[i].x - pose[0];
        dum.y = bound[i].y - pose[1];
        temp.x = ct*dum.x - st*dum.y;  // interchange the x and y
        temp.y = st*dum.x + ct*dum.y;
        row_pix = temp.x/resolution_m;
        column_pix = (temp.y + 0.5*costmap_width_m)/resolution_m;
        cv::circle(bound_map, cv::Point(column_pix, row_pix), 0.2/resolution_m, cv::Scalar(1.0), -1);
      }
    }

    map += bound_map;

    cv::Mat gradmap_new;
    cv::GaussianBlur(gradmap, gradmap_new, cv::Size(filt_size, filt_size), 0, 0, cv::BORDER_DEFAULT);
    // // Update GUI Window
    // cv::Point p1(1/resolution_m + costmap_width_p/2, 1/resolution_m);
    // // Bottom Right Corner
    // cv::Point p2(-1/resolution_m + costmap_width_p/2, 0);
  
    // Drawing the Rectangle
    // cv::rectangle(map, p1, p2,
    //           1,
    //           2, cv::LINE_8);
    cv::Mat display;
    cv::flip(map, display, -1);
    cv::Mat grad_disp;
    cv::flip(gradmap_new, grad_disp, -1);

    float delta_time = (ros::Time::now() - begin).toSec();
    ROS_INFO("%f", delta_time*1000);
    // cv::imshow("gradmap", grad_disp);
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