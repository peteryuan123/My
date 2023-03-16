//
// Created by mpl on 22-8-30.
//

#ifndef MYSLAM_IMUFRAME_H
#define MYSLAM_IMUFRAME_H

#include "FrameBase.h"
#include <Eigen/Core>

class ImuFrame: public FrameBase
{
public:
    ImuFrame(const Eigen::Vector3d &acc, const Eigen::Vector3d &acc_b,
             const Eigen::Vector3d &gyro, const Eigen::Vector3d &gyro_b, double timeStamp)
             : FrameBase(timeStamp), m_acc(acc), m_acc_b(acc_b), m_gyro(gyro), m_gyro_b(gyro_b) {}

    Eigen::Vector3d m_acc;
    Eigen::Vector3d m_acc_b;
    Eigen::Vector3d m_gyro;
    Eigen::Vector3d m_gyro_b;

};

#endif //MYSLAM_IMUFRAME_H
