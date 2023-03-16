//
// Created by mpl on 22-7-14.
//

#ifndef MYSLAM_PINHOLECAMERA_H
#define MYSLAM_PINHOLECAMERA_H

#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "CameraBase.h"


class PinholeCamera: public CameraBase
{
public:
    typedef std::shared_ptr<PinholeCamera> Ptr;

    PinholeCamera(const cv::FileNode &sensorNode);

    inline bool isPreUndistort() const
    {
        return m_isPreUndistort;
    }

    inline Eigen::Vector2d project(const Eigen::Ref<const Eigen::Vector3d> &Xc) override
    {
        Eigen::Vector3d p = m_K * Xc;
        return p.segment(0, 2) / p[2];
    }

    inline Eigen::Vector3d invProject(const Eigen::Ref<const Eigen::Vector2d> &x) override
    {
        Eigen::Vector3d p_homo(x[0], x[1], 1);
        Eigen::Vector3d Xc = m_invK * p_homo;
        return Xc.normalized();
    }


    void undistortImage(const cv::Mat& src, cv::Mat& dest);
    void undistortPoints(const cv::Mat& src, cv::Mat& dest);

protected:
    Eigen::Matrix3d m_K;   // maybe redundent with "m_parameter" in base class
    Eigen::Matrix3d m_invK;
    Eigen::Vector2d m_size;

    // UndistortMap
    bool m_isPreUndistort;
    cv::Mat m_M1, m_M2; // not used if m_isPreUndistort is set to false;

    DistortionModel m_distortType;
};

//TODO: seems ORB did not use distortion coeff


#endif //MYSLAM_PINHOLECAMERA_H
