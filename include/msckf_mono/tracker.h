#ifndef MSCKF_MONO_ROS_TRACKER_H_
#define MSCKF_MONO_ROS_TRACKER_H_

#pragma once

// Standard
#include <vector>
#include <map>
#include <unordered_map>
#include <stdlib.h>
#include <math.h>
#include <iostream>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/video/tracking.hpp"

// Fast corner detection
#include <fast/fast.h>
#include <msckf_mono/types.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>



// Eigen
#include <Eigen/Dense>

// Matrix utils & types
#include <msckf_mono/matrix_utils.h>



typedef cv::Point2f Point2f; 
typedef std::vector<cv::Point2f> Point2fVector;
typedef std::vector<size_t> IdVector;
//typedef std::vector<msckf_mono::Vector2<float>,
//                    Eigen::aligned_allocator<msckf_mono::Vector2<float>>>
//                      OutFeatureVector;

class imuCamState_t{
public:
    imuCamState_t(ros::NodeHandle n);
    ros::NodeHandle nh_;
    void imuCallback( const sensor_msgs::Imu::ConstPtr& msg);
    void cameraCallback(const sensor_msgs::ImageConstPtr &msg);
    msckf_mono::imuReading<float> curr_;
    cv::Mat img_raw;



};


class boundingBoxState_t{
public:
    boundingBoxState_t();
    void init_node (ros::NodeHandle n);
    ros::NodeHandle nh_;
   // std::vector<msckf_mono::imgBboxes<float>> img_bboxes_states_;
    msckf_mono::imgBboxes<float> img_bboxes;
    cv::Mat img_raw;
    float img_w_, img_h_;

   // msckf_mono::bbox<float> one_bbox;
   // void cameraCallback(const sensor_msgs::ImageConstPtr &msg);

   // ros::Subscriber cam_sub = nh_.subscribe("detections", 200, &boundingBoxState_t::cameraCallback, this);

    //    message_filters::Subscriber<boundingBoxState_t> bb_sub(nh, "bounding_boxes", 1);
    //    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "image_bbox", 1);

};


class bbTracker_t
{
public:

    bbTracker_t(ros::NodeHandle nh);
    ros::NodeHandle nh;
    void boundingboxesCallback(const darknet_ros_msgs::BoundingBoxes &bboxes_ros);
    ros::Subscriber bb_sub_= nh.subscribe("bounding_boxes", 70, &bbTracker_t::boundingboxesCallback, this);

//never change
    msckf_mono::Quaternion<float > world_orig_q_;
    msckf_mono::Vector3<float >world_orig_p_;
//update with time
    msckf_mono::imuState<float> curr_imu_state_;
    msckf_mono::Matrix3<float> R_imu_cam_;
    msckf_mono::Camera<float> camera_;
    float z_C;



    cv::Mat last_detected_frame;
    int max_age, min_hit, frame_count;
    msckf_mono::imgBboxes<float> last_detected_bbox;
    msckf_mono::imgBboxes<float> new_predicted_bbox;

    std::vector<msckf_mono::bboxState<float>> bbox_State_vect;
    std::vector< cv::Mat> frame_queue;
    std::vector< msckf_mono::imgBboxes<float>> detected_bboxes_queue;
    boundingBoxState_t bb_state_;

   void init( msckf_mono::Vector3<float >curr_pos_p,
              msckf_mono::Quaternion<float> curr_pos_q, msckf_mono::Camera<float> camera, msckf_mono::Matrix3<float> R_imu_cam);
   void update_pose(msckf_mono::imuState<float> imu_state);
   void newBB(msckf_mono::imgBboxes<float> img_bboxes);
   void project_pixel_to_world(  msckf_mono::ray<float>& tl_ray, msckf_mono::ray<float>& br_ray,
                                msckf_mono::bbox<float> &detected_bbox );
   void project_world_to_pixel(  msckf_mono::bboxState<float> bbox_state,
                                msckf_mono::bbox<float> &predicted_bbox );

};




//class CornerTracker
//{
//  public:
//    CornerTracker(int window_size=51,
//                  double min_eigen_threshold=0.001,
//                  int max_level=5,
//                  int termcrit_max_iters=50,
//                  double termcirt_epsilon=0.01);

//    ~CornerTracker() {};

//    void configure(double window_size,
//                   double min_eigen_threshold,
//                   int max_level,
//                   int termcrit_max_iters,
//                   double termcirt_epsilon);

//    void track_features(cv::Mat img_1, cv::Mat img_2, Point2fVector& points1, Point2fVector& points2, IdVector& id1, IdVector& id2);

//  private:
//    cv::Size window_size_;

//    double min_eigen_threshold_;
//    int max_level_;

//    cv::TermCriteria termination_criteria_;
//};

//class TrackVisualizer
//{
//  public:
//    TrackVisualizer();

//    void add_current_features(Point2fVector& features, IdVector& feature_ids);
//    void add_new_features(Point2fVector& features, IdVector& feature_ids);
//    void add_predicted(Point2fVector& features, IdVector& feature_ids);

//    cv::Mat draw_tracks(cv::Mat img);

//  private:
//    std::unordered_map<int, Point2fVector> feature_tracks_;
//    std::unordered_map<int, Point2f> predicted_pts_;
//};

//class TrackHandler
//{
//  public:
//    TrackHandler(const cv::Mat K, const cv::Mat dist_coeffs, const std::string dist_model);
//    ~TrackHandler();

//    void add_gyro_reading(Eigen::Vector3f& gyro_reading);

//    void set_current_image(cv::Mat img, double time);
//    void tracked_features(OutFeatureVector& features, IdVector& feature_ids);
//    void new_features(OutFeatureVector& features, IdVector& feature_ids);

//    void clear_tracks();
//    size_t get_next_feature_id(){return next_feature_id_;}

//    Point2fVector get_prev_features(){return prev_features_;}
//    IdVector get_prev_ids(){return prev_feature_ids_;}

//    void set_grid_size(int n_rows, int n_cols);
//    void set_ransac_threshold(double rt);

//    cv::Mat get_track_image();

//  private:
//    Eigen::Array<bool, 1, Eigen::Dynamic>
//    twoPointRansac(const msckf_mono::Matrix3<float>& dR,
//                   const OutFeatureVector& old_points_in,
//                   const OutFeatureVector& new_points_in);

//    void undistortPoints(Point2fVector& in, Point2fVector& out);

//    void integrate_gyro();
//    void predict_features();

//    cv::Mat dR_;

//    double ransac_threshold_;
//    CornerDetector detector_;
//    CornerTracker tracker_;
    
//    double cur_time_;
//    cv::Mat cur_img_;
//    Point2fVector cur_features_;
//    IdVector cur_feature_ids_;

//    Point2fVector new_features_;
//    IdVector new_feature_ids_;

//    double prev_time_;
//    cv::Mat prev_img_;
//    Point2fVector prev_features_;
//    IdVector prev_feature_ids_;

//    size_t next_feature_id_;

//    Eigen::Vector3f gyro_accum_;
//    size_t n_gyro_readings_;
//    bool use_gyro_;

//    const std::string distortion_model_;
//    const cv::Mat distortion_coeffs_;
//    const cv::Mat K_;
//    const cv::Mat K_inv_;

//    TrackVisualizer visualizer_;
//};

#endif
