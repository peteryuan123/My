//
// Created by mpl on 22-7-22.
//

#ifndef MYSLAM_FRAMEBASE_H
#define MYSLAM_FRAMEBASE_H

#include <memory>

class FrameBase
{
public:
    typedef std::shared_ptr<FrameBase> Ptr;

    FrameBase(double timeStamp):
    m_timeStamp(timeStamp) {};

    ~FrameBase();

protected:
    // Todo: add posez
    double m_timeStamp;
};

#endif //MYSLAM_FRAMEBASE_H
