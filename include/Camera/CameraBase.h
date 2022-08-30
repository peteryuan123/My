//
// Created by mpl on 22-7-19.
//

#ifndef MYSLAM_CAMERABASE_H
#define MYSLAM_CAMERABASE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <opencv2/opencv.hpp>

class CameraBase
{
public:
    typedef std::shared_ptr<CameraBase> Ptr;

    CameraBase():
    m_camId(0), m_Qsb(Eigen::Quaterniond::Identity()), m_tsb(Eigen::Vector3d::Zero()) {};

    CameraBase(Eigen::VectorXd parameter):
    m_camId(0), m_parameter(parameter),m_Qsb(Eigen::Quaterniond::Identity()), m_tsb(Eigen::Vector3d::Zero()) {};

    CameraBase(Eigen::VectorXd parameter, const Eigen::Quaterniond& Qsb, const Eigen::Ref<const Eigen::Vector3d> tsb):
    m_camId(0), m_parameter(parameter), m_Qsb(Qsb), m_tsb(tsb) {};

    CameraBase(const cv::FileNode &sensorNode)
    {
        cv::FileNode data;
        std::vector<double> temp;

        m_camId = sensorNode["id"];

        // Extrinsic
        data = sensorNode["Qsb"];
        for (cv::FileNodeIterator dataItr = data.begin(); dataItr != data.end(); dataItr++){
            temp.emplace_back(*dataItr);
        }
        m_Qsb = Eigen::Quaterniond(temp.data());
        temp.clear();

        data = sensorNode["tsb"];
        for (cv::FileNodeIterator dataItr = data.begin(); dataItr != data.end(); dataItr++){
            temp.emplace_back(*dataItr);
        }
        m_tsb = Eigen::Vector3d(temp.data());
        temp.clear();


        // Extrinsic

    }


    virtual inline Eigen::Vector2d project(const Eigen::Ref<const Eigen::Vector3d> &Xc) = 0;

    virtual inline Eigen::Vector3d invProject(const Eigen::Ref<const Eigen::Vector2d> &x) = 0;

    inline Eigen::Quaterniond& Qsb()
    {
        return m_Qsb;
    }

    inline Eigen::Vector3d& tsb()
    {
        return m_tsb;
    }

    inline void setQsb(const Eigen::Quaterniond &Q)
    {
        m_Qsb = Q;
    }

    inline void settsb(const Eigen::Vector3d &t)
    {
        m_tsb = t;
    }

    virtual ~CameraBase() = default;

protected:
    int m_camId;
    Eigen::Quaterniond m_Qsb; // body to sensor
    Eigen::Vector3d m_tsb;
    Eigen::VectorXd m_parameter;
};


#endif //MYSLAM_CAMERABASE_H
