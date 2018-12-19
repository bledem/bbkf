#ifndef MSCKF_MONO_SENSOR_TYPES_H_
#define MSCKF_MONO_SENSOR_TYPES_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

namespace msckf_mono {
  template <typename _Scalar>
    using Quaternion = Eigen::Quaternion<_Scalar>;

  template <typename _Scalar>
    using Matrix3 = Eigen::Matrix<_Scalar, 3, 3>;

  template <typename _Scalar>
    using Matrix4 = Eigen::Matrix<_Scalar, 4, 4>;

  template <typename _Scalar>
    using MatrixX = Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic>;

  template <typename _Scalar>
    using RowVector3 = Eigen::Matrix<_Scalar, 1, 3>;

  template <typename _Scalar>
    using Vector2 = Eigen::Matrix<_Scalar, 2, 1>;

  template <typename _Scalar>
    using Vector3 = Eigen::Matrix<_Scalar, 3, 1>;

  template <typename _Scalar>
    using Vector4 = Eigen::Matrix<_Scalar, 4, 1>;

  template <typename _Scalar>
    using VectorX = Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>;

  template <typename _Scalar>
    using Point = Vector3<_Scalar>;

  template <typename _Scalar>
    using GyroscopeReading = Vector3<_Scalar>;

  template <typename _Scalar>
    using AccelerometerReading = Vector3<_Scalar>;

  template <typename _Scalar>
    using Isometry3 = Eigen::Transform<_Scalar,3,Eigen::Isometry>;

  template <typename _Scalar>
    struct Camera {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        _Scalar c_u, c_v, f_u, f_v, b;

      Quaternion<_Scalar> q_CI;
      Point<_Scalar> p_C_I;
int img_w;
int img_h;
    };

  template <typename _Scalar>
    struct camState {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Point<_Scalar> p_C_G;
      Quaternion<_Scalar> q_CG;
      _Scalar time;
      int state_id;
      int last_correlated_id;
      std::vector<size_t> 		tracked_feature_ids;

    };

//Added BB
template <typename _Scalar>
    struct ray {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

       Point<_Scalar> p_GR;
      _Scalar time;
      int state_id;
      int last_correlated_id;
      std::vector<size_t> tracked_feature_ids;
    };

template <typename _Scalar>
  struct bbox {
  float xmin, ymin, xmax, ymax, prob;

  int num;
 std::string Class;
};

template <typename _Scalar>
    struct bboxState {
	 size_t bbox_id; 
        size_t feature_id[4]= {0,0,0,0}; //track id
        ray<_Scalar> r_tl, r_br;
	Vector3<_Scalar> p_f_G_tl, p_f_G_br ;
      _Scalar time;
//last YOLO detection values (not updated)
        bbox<_Scalar> prev_detection, cur_detection;
        int age; //for every new object
        float prev_time_detection;
      std::string Class;
	bool associated;
    };



template <typename _Scalar>
  struct imgBboxes {

  std::vector<msckf_mono::bbox<_Scalar>> list;
  float time;
};

//end Added

  template <typename _Scalar>
    struct imuState {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      Point<_Scalar> p_I_G, p_I_G_null;
      Vector3<_Scalar> v_I_G, b_g, b_a, g, v_I_G_null;
      Quaternion<_Scalar> q_IG, q_IG_null;
    };

  template <typename _Scalar>
    struct imuReading {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        GyroscopeReading<_Scalar> omega;
      AccelerometerReading<_Scalar> a;
      _Scalar dT;
    };

  template <typename _Scalar>
    struct noiseParams {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        _Scalar u_var_prime, v_var_prime;
      Eigen::Matrix<_Scalar, 12, 12> Q_imu;
      Eigen::Matrix<_Scalar, 15, 15> initial_imu_covar;
    };

  template <typename _Scalar>
    struct MSCKFParams {
      _Scalar max_gn_cost_norm, min_rcond, translation_threshold;
      _Scalar redundancy_angle_thresh, redundancy_distance_thresh;
      int min_track_length, max_track_length, max_cam_states;
    };

  template <typename _Scalar>
    struct featureTrackToResidualize {
      size_t feature_id;
      std::vector<Vector2<_Scalar>,
        Eigen::aligned_allocator<Vector2<_Scalar>>> observations;

      std::vector<camState<_Scalar>> cam_states;
      std::vector<size_t> cam_state_indices;

      bool initialized;
      Vector3<_Scalar> p_f_G;

      featureTrackToResidualize() : initialized(false) {}
    };

  template <typename _Scalar>
    struct featureTrack {
      size_t feature_id;
      std::vector<Vector2<_Scalar>,
        Eigen::aligned_allocator<Vector2<_Scalar>>> observations;

      std::vector<size_t> cam_state_indices; // state_ids of cam states corresponding to observations

      bool initialized = false;
      Vector3<_Scalar> p_f_G;
    };
}

#endif
