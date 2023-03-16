//
// Created by mpl on 22-8-29.
//

#include <Eigen/Core>
#include <iostream>
#include <vector>

#include "Tool/NoiseGenerator.h"

typedef Eigen::Matrix3d rotation_t;
typedef Eigen::Vector3d translation_t;

struct MotionData
{
    rotation_t Rwb;
    translation_t twb;
};

void add_noise_to_motion(MotionData& data)
{

//    Eigen::Vector3d::Random()
//    data.Rwb =
}

// along y-axis
void generateStraightTrajectory(std::vector<MotionData>& trajectory, MotionData start_point,
                                double offset, int n)
{

}

void generatePureRotationTrajectory()
{

}

int main(int argc, char** argv)
{
    NoiseGenerator generator(NoiseGenerator::randomType::GAUSSIAN, 0, 1);
    for (int i= 0 ; i < 10000; i++)
    {
        std::cout << generator.generateNoise() << std::endl;
    }
    return 0;
}
