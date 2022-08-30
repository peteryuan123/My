//
// Created by mpl on 22-7-14.
//

#include "Camera/PinholeCamera.h"
#include <opencv2/core/eigen.hpp>

// CameraBase read id and extrinsic
PinholeCamera::PinholeCamera(const cv::FileNode &sensorNode):
CameraBase(sensorNode), m_K(Eigen::Matrix3d::Zero()), m_invK(Eigen::Matrix3d::Zero())
{
    cv::FileNode data;
    std::string distortion_type;
    std::vector<double> temp;

    data = sensorNode["size"];
    for (cv::FileNodeIterator dataItr = data.begin(); dataItr != data.end(); dataItr++){
        m_size(dataItr - data.begin()) = *dataItr;
    }

    data = sensorNode["distortion_type"];
    distortion_type = data.string();
    if (data.string() == "equidistant")
        m_parameter.resize(8); // fx, fy, cx, cy, k1, k2, k3, k4
    else if (data.string() == "pinhole")
        m_parameter.resize(9); // fx, fy, cx, cy, k1 k2, p1, p2, k3
    else
        throw std::invalid_argument("not support type:" + data.string());

    //TODO: check invalid input

    // Intrinsic
    int counter = 0;
    data = sensorNode["focal_length"];
    for (cv::FileNodeIterator dataItr = data.begin(); dataItr != data.end(); dataItr++){
        m_parameter[counter] = *dataItr;
        counter++;
    }
    data = sensorNode["principal_point"];
    for (cv::FileNodeIterator dataItr = data.begin(); dataItr != data.end(); dataItr++){
        m_parameter[counter] = *dataItr;
        counter++;
    }
    data = sensorNode["distortion"];
    for (cv::FileNodeIterator dataItr = data.begin(); dataItr != data.end(); dataItr++){
        m_parameter[counter] = *dataItr;
        counter++;
    }
    m_K << m_parameter[0], 0,  m_parameter[2],
            0, m_parameter[1], m_parameter[3],
            0, 0, 1;
    // Intrinsic


    // preUndistort (Currently only support preUndistort)
    if (distortion_type == "equidistant")
    {
        cv::Mat K;
        cv::eigen2cv(m_K, K);
        cv::Mat1f D = (cv::Mat1f(1, 4) << m_parameter[4], m_parameter[5], m_parameter[6], m_parameter[7]);
        cv::Size cvSize(m_size[0], m_size[1]);

        cv::Mat P;
        // TODO: test P
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, cvSize, cv::Mat::eye(3, 3, CV_32F), P, 1);
        cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat::eye(3, 3, CV_32F), P, cvSize, CV_32F, m_M1, m_M2);

        // 1
        cv::cv2eigen(P, m_K);
        // 2
//        Eigen::Matrix<double, 3, 4> Pr;
//        cv::cv2eigen(P, Pr);
//        m_K = Pr.block<3, 3>(0, 0);

        m_invK = m_K.inverse();
    }
    else if (distortion_type == "pinhole")
    {
        cv::Mat K;
        cv::eigen2cv(m_K, K);
        cv::Mat1f D = (cv::Mat1f(1, 5) << m_parameter[4], m_parameter[5], m_parameter[6], m_parameter[7], m_parameter[8]);
        cv::Size cvSize(m_size[0], m_size[1]);

        cv::Mat new_K = cv::getOptimalNewCameraMatrix(K, D, cvSize, 0);
        cv::initUndistortRectifyMap(K, D, cv::Mat::eye(3, 3, CV_32F), new_K, cvSize, CV_32F, m_M1, m_M2);

        cv::cv2eigen(new_K, m_K);
        m_invK = m_K.inverse();

    }
    std::cout << "************Pinhole Camera**********\n";
    std::cout << "size_:" << m_size.transpose()<< std::endl;
    std::cout << "original parameter (fx, fy, cx, cy, distortion):" << m_parameter.transpose()<< std::endl;
    std::cout << "Undistort_K:" << m_K << std::endl;
    std::cout << "m_Qsb:" << m_Qsb.coeffs().transpose() << std::endl;
    std::cout << "m_tsb:" << m_tsb.transpose() << std::endl;

}


void PinholeCamera::undistortImage(const cv::Mat& src, cv::Mat& dest)
{
    cv::remap(src, dest, m_M1, m_M2, cv::INTER_AREA);
}
