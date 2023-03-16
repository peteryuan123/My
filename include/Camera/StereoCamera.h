
#ifndef MYSLAM_STEREOCAMERA_H
#define MYSLAM_STEREOCAMERA_H

#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "CameraBase.h"

#include "Tool/Timer.h"

// StereoCamera only suppot: PreUndistort
class StereoCamera: public CameraBase
{
public:
    typedef std::shared_ptr<StereoCamera> Ptr;

//    StereoCamera(const Eigen::Ref<const Eigen::Vector2d>& size,
//                 const Eigen::Ref<const Eigen::Matrix3d>& left_K, const Eigen::Ref<const Eigen::Matrix3d>& right_K,
//                 const Eigen::Ref<const Eigen::Matrix3d>& R_rl, const Eigen::Ref<const Eigen::Vector3d>& t_rl,
//                 const Eigen::Ref<const Eigen::VectorXd> &left_distCoeffs = Eigen::VectorXd(),
//                 const Eigen::Ref<const Eigen::VectorXd> &right_distCoeffs = Eigen::VectorXd());

    StereoCamera(const cv::FileNode &sensorNode);

    inline Eigen::Vector2d project(const Eigen::Ref<const Eigen::Vector3d> &Xc) override
    {
        Eigen::Vector3d Xuv = m_K * Xc;
        return Xuv.segment(0, 2) / Xuv[2];
    }

    inline Eigen::Vector3d invProject(const Eigen::Ref<const Eigen::Vector2d> &x) override
    {
        Eigen::Vector3d p_homo(x[0], x[1], 1);
        p_homo = m_invK * p_homo;
        return p_homo.normalized();
    }

    inline Eigen::Vector2d project_right(const Eigen::Ref<const Eigen::Vector3d> &Xc)
    {
        Eigen::Vector3d Xuv = m_K * Xc;
        Xuv[0] = Xuv[0] + m_stereo_shift;
        return Xuv.segment(0, 2) / Xuv[2];
    }

    // After rectification the
    inline Eigen::Vector3d triangulate(const Eigen::Ref<const Eigen::Vector2d> &uv_left,
                                       const Eigen::Ref<const Eigen::Vector2d> &uv_right)
    {
        Eigen::Vector4d uvpoint(uv_left[0], uv_left[1], uv_left[0]-uv_right[0], 1);
        Eigen::Vector4d cam_point = m_Q * uvpoint;
        return cam_point.segment(0, 3) / cam_point[3];
    }

    void undistortRectify(const cv::Mat& src_l, const cv::Mat& src_r, cv::Mat& dest_l, cv::Mat& dest_r);



public:
    Eigen::Vector2d m_size;
    Eigen::Matrix3d m_K; // The two intrinsic should be equal
    Eigen::Matrix3d m_invK;
    double m_stereo_shift;  // https://docs.opencv.org/4.2.0/d9/d0c/group__calib3d.html#ga617b1685d4059c6040827800e72ad2b6
    Eigen::Matrix4d m_Q; // reprojection matrix used in triangulation

    // The rotation should be identity, so we don't save it here
    Eigen::Vector3d m_trl;

    // rectified image has this map
    cv::Mat m_M1l, m_M2l;
    cv::Mat m_M1r, m_M2r;
};

#endif //MYSLAM_STEREOCAMERA_H
