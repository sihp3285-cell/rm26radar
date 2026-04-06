#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <opencv2/opencv.hpp>
#include <deque>
#include <map>
#include <cmath>
#include <vector>
#include "radarmap.hpp"


struct Target
{
    cv::Point2f mappoint;
    std::deque<int> classhistory;
    int missCount = 0;
    int color;
};

class Tracker
{
    private:
    std::vector<Target> targets;
    int maxMissCount;
    int maxhistory;
    float distheshold;
    public:

    Tracker(int maxMissCount, int maxhistory, float distheshold);
    void update(const std::vector<Mappoint>& detections);
    std::vector<Mappoint> getSmoothedPoints(); 
};

#endif