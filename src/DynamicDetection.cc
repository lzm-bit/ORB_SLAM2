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


#include "DynamicDetection.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Converter.h"
#include "LocalMapping.h"

#include<mutex>
#include<chrono>

namespace ORB_SLAM2
{

DynamicDetection::DynamicDetection(System* pSys, Map* pMap):
    mpSystem(pSys), mpMap(pMap), mbResetRequested(false), mbResetRequestedActiveMap(false), mbFinishRequested(false), mbFinished(true),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true), mpCurrentKeyFrame(nullptr), weight_ba_(false)
{ }

void DynamicDetection::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void DynamicDetection::Run(KeyFrame* kf)
{
    if (kf == nullptr || mpMap->KeyFramesInMap()<=2) {
        // weight_ba_ = false;
        return;
    }

    // Local BA
    // mbAbortBA = false;
    Optimizer::UMLocalBundleAdjustment(kf,&mbAbortBA, mpMap);
    usleep(500);
}

void DynamicDetection::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
    // std::cout << "ORB-SLAM3 obtain kf timestamp: " 
    //       << std::fixed << std::setprecision(6) // 设置定点表示，保留 6 位小数
    //       << pKF->mTimeStamp << std::endl;
}

bool DynamicDetection::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

KeyFrame* DynamicDetection::GetCurrKF()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    if (mlNewKeyFrames.empty()) {
        return nullptr;
    }

    mpCurrentKeyFrame = mlNewKeyFrames.front();
    mlNewKeyFrames.pop_front();
    return mpCurrentKeyFrame;
}

const std::vector<std::vector<Eigen::Vector3f>>& DynamicDetection::GetDynamicBoxes() {
    unique_lock<mutex> lock(mMutexNewUnMap);
    return dynamic_boxes;
}

KeyFrame* DynamicDetection::GetCurrKF(const double& timestamp) {
    unique_lock<mutex> lock(mMutexNewUnMap);
    const std::unordered_map<double, KeyFrame*> all_kfs = mpMap->GetAllKeyFramesMap();
    auto kf_iter = all_kfs.find(timestamp);
    if (kf_iter == all_kfs.end()) {
        std::cout << "error not find this keyframes: " << timestamp << std::endl;
        return nullptr;
    }

    KeyFrame* kf = kf_iter->second;
    if (!kf || kf->isBad()) {
        return nullptr;
    }
    return kf;
}

void DynamicDetection::VisualKeyPoints(KeyFrame* kf) {
    const int n = kf->mvKeys.size();
    auto mvbMap = vector<bool>(n,false);
    for(int i=0;i<n;i++)
    {
        MapPoint* pMP = kf->GetMapPoint(i);
        if(pMP)
        {
            if(pMP->Observations()>0)
                mvbMap[i]=true;
        }
    }

    const float r = 5;
    
    auto vCurrentKeys = kf->mvKeys;
    cv::Mat im = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));;
    kf->mImColor.copyTo(im);
    for(int i=0;i<n;i++)
    {
        cv::Point2f pt1,pt2;
        pt1.x=vCurrentKeys[i].pt.x-r;
        pt1.y=vCurrentKeys[i].pt.y-r;
        pt2.x=vCurrentKeys[i].pt.x+r;
        pt2.y=vCurrentKeys[i].pt.y+r;

        auto weight = kf->GetFeatureWeight(i);
        // std::cout << "weight: " << weight << std::endl;
        if(weight > 0.5)
        {
            cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
            cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
        }
        else 
        {
            cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
            cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
        }
    }
    
    // cv::namedWindow("ORB-SLAM2: Uncertainty Map Frame");
    cv::imshow("ORB-SLAM2: Current Frame",im);
    // cv::waitKey(1.0);
}

void DynamicDetection::SetHistoryUncertaintyMap(const std::map<double, cv::Mat>& history_maps) {
    for (const auto& [timestamp, uncertainty_map] : history_maps) {
        KeyFrame* kf = GetCurrKF(timestamp);
        if (kf == nullptr) {
            return;
        }

        // uncertainty_maps_.insert({timestamp, std::make_tuple(kf, uncertainty_map)}); // TODO ...
        cv::Mat save_map;
        if (uncertainty_map.type() == CV_32FC1) {
            uncertainty_map.convertTo(save_map, CV_8UC1, 255.0);
        } else if (uncertainty_map.type() == CV_8UC1) {
            save_map = uncertainty_map;
        } else {
            std::cerr << "[DynamicDetection] Warning: unsupported uncertainty_map type: "
                      << uncertainty_map.type() << ", skip saving." << std::endl;
            save_map = cv::Mat();
        }

        // if (!save_map.empty()) {
        //     std::string path =
        //         "/home/lzm/Project2/Open-Vocabulary-dynamic-Scene-Graph/results/"
        //         + std::to_string(timestamp) + ".png";
        //
        //     bool ok = cv::imwrite(path, save_map);
        //     if (!ok) {
        //         std::cerr << "[DynamicDetection] Warning: failed to save "
        //                   << path << std::endl;
        //     }
        // }

        std::unordered_map<size_t, float> uncertainty_indexs = kf->UpdateUncertaintyWeights(uncertainty_map);
        kf->UpdateUncertaintyMapPoints(uncertainty_indexs);
    }
}

void DynamicDetection::SetDynamicBox(const std::vector<std::vector<Eigen::Vector3f>>& boxes) {
    unique_lock<mutex> lock(mMutexNewUnMap);
    dynamic_boxes = boxes;
}

void DynamicDetection::SetUncertaintyMap(const double& timestamp, const cv::Mat& uncertainty_map) {
    KeyFrame* kf = GetCurrKF(timestamp);
    if (kf == nullptr) {
        return;
    }

    uncertainty_maps_.insert({timestamp, std::make_tuple(kf, uncertainty_map)});

    std::unordered_map<size_t, float> uncertainty_indexs = kf->UpdateUncertaintyWeights(uncertainty_map);
    kf->UpdateUncertaintyMapPoints(uncertainty_indexs);
    kf->UpdateConnections();
    Run(kf);

    VisualKeyPoints(kf);
    // std::unique_lock<std::mutex> map_lock(mpMap->mMutexMapUpdate);
    // const std::vector<cv::KeyPoint>& vKeysUn = kf->mvKeysUn;
    // const int N = kf->N;

    // int erased_mp_count = 0;
    // const float threshold = 0.5f;
    // for (int i = 0; i < N; ++i) {
    //     MapPoint* pMP = kf->GetMapPoint(i);
    //     if (!pMP || pMP->isBad()) {
    //         continue;
    //     }

    //     int u = cvRound(vKeysUn[i].pt.x);
    //     int v = cvRound(vKeysUn[i].pt.y);

    //     if (u < 0 || u >= uncertainty_map.cols || v < 0 || v >= uncertainty_map.rows) {
    //         continue;
    //     }

    //     float uncertainty = 0.0f;
    //     if (uncertainty_map.type() == CV_32FC1) {
    //         uncertainty = uncertainty_map.at<float>(v, u);
    //     } else if (uncertainty_map.type() == CV_8UC1) {
    //         uncertainty = uncertainty_map.at<uint8_t>(v, u) / 255.0f;
    //     }

    //     if (uncertainty > threshold) {
    //         kf->EraseMapPointMatch(i);
    //         pMP->SetBadFlag();

    //         // kf->mvbOutlier[i] = true;

    //         erased_mp_count++;
    //     }
    // }
    // std::cout << "[C++] KF ID: " << kf->mnId  << " | Erased " << erased_mp_count << " dynamic MapPoints." << std::endl;
}

} //namespace ORB_SLAM
