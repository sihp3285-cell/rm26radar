#include "include/tracker.hpp"

Tracker::Tracker(int maxMissCount, int maxhistory, float distheshold)
{
    this->maxMissCount = maxMissCount;
    this->maxhistory = maxhistory;
    this->distheshold = distheshold;
}

void Tracker::update(const std::vector<Mappoint>& detections)
{
    for(auto& t : targets)
    {
        t.missCount++;
    }
    for(const auto& d : detections)
    {
        Target* bestMatch = nullptr;
        float bestDist = distheshold;
        for(auto& t : targets)
        {
            float dist = cv::norm(t.mappoint - d.map_point);
            if(dist < bestDist)
            {
                bestDist = dist;
                bestMatch = &t;
            }
        }
        if(bestMatch)
        {
            bestMatch->missCount = 0;
            bestMatch->mappoint = d.map_point;
            bestMatch->classhistory.push_back(d.classIdx);
            if(bestMatch->classhistory.size() > maxhistory)
            {
                bestMatch->classhistory.pop_front();
            }
            bestMatch->color = d.armorColor;
        }
        else
        {
            Target newTarget;
            newTarget.mappoint = d.map_point;
            newTarget.classhistory.push_back(d.classIdx);
            newTarget.missCount = 0;
            newTarget.color = d.armorColor;
            targets.push_back(newTarget);
        }
    }
    targets.erase(std::remove_if(targets.begin(), targets.end(), [this](const Target& t){
        return t.missCount > maxMissCount;
    }), targets.end());
}

std::vector<Mappoint> Tracker::getSmoothedPoints()
{
    std::vector<Mappoint> Result;
    for(const auto& t : targets)
    {
        int bestclass = t.classhistory.back();
        int maxcount = 0;
        std::map<int,int> count;
        for(const auto& c : t.classhistory)
        {
            count[c]++;
            if(count[c] > maxcount)
            {
                maxcount = count[c];
                bestclass = c;
            }
        }
        Result.push_back({t.mappoint, "", bestclass, t.color});
    }
    return Result;
}
