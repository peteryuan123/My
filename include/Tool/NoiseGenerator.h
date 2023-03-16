//
// Created by mpl on 22-8-30.
//

#ifndef MYSLAM_NOISEGENERATOR_H
#define MYSLAM_NOISEGENERATOR_H

#include <random>

class NoiseGenerator
{
public:
    enum randomType {
        GAUSSIAN,
    };

    NoiseGenerator(randomType type, double mean, double cov):
    m_rd(), m_gen({m_rd()}), m_type(type), m_d(mean, cov)
    {};

    inline double generateNoise()
    {
        return m_d(m_gen);
    }

private:
    randomType m_type;
    std::random_device m_rd;
    std::mt19937 m_gen;

    std::normal_distribution<> m_d;

};


#endif //MYSLAM_NOISEGENERATOR_H
