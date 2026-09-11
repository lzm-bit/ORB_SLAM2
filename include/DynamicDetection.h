/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef DYNAMICDETECTION_H
#define DYNAMICDETECTION_H

#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include <map>

#include <mutex>


namespace ORB_SLAM2
{

class Map;
class System;
class Tracking;
class LoopClosing;

class DynamicDetection
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DynamicDetection(System* pSys, Map* pMap);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run(KeyFrame* kf);

    void InsertKeyFrame(KeyFrame* pKF);
  
    KeyFrame* GetCurrKF();
    KeyFrame* GetCurrKF(const double& timestamp);

    void SetUncertaintyMap(const double& timestamp, const cv::Mat& uncertainty_map);
    void SetHistoryUncertaintyMap(const std::map<double, cv::Mat>& history_maps);
    void SetDynamicBox(const std::vector<std::vector<Eigen::Vector3f>>& dynamic_boxes);
    const std::vector<std::vector<Eigen::Vector3f>>& GetDynamicBoxes();
    double mFirstTs;
    
protected:    
    // bool AcceptKeyFrames();
    // void SetAcceptKeyFrames(bool flag);
    //
    // bool SetNotStop(bool flag);

    bool CheckNewKeyFrames();
    // void ProcessNewKeyFrame();
    System *mpSystem;

    Map* mpMap;

    bool mbMonocular;
    bool mbInertial;

    // void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    // bool CheckFinish();
    // void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;
    std::unordered_map<double, std::tuple<KeyFrame*, cv::Mat>> uncertainty_maps_;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    std::mutex mMutexNewUnMap;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    bool weight_ba_;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    private:

    void VisualKeyPoints(KeyFrame* kf);

    std::vector<std::vector<Eigen::Vector3f>> dynamic_boxes;

};

} //namespace ORB_SLAM

#endif // DYNAMICDETECTION_H