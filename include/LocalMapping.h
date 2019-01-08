/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>
#include <condition_variable>
#include <atomic>

namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

class LocalMapping
{
public:
    LocalMapping(Map* pMap, const float bMonocular, string linearSolverName, string optimizationAlgorithmName);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread state manipulation signals
    void RequestStop();
    void RequestRelease();
    void RequestFinish();

    //Specific signals
    void RequestReset();
    void RequestInterruptBA();
    bool NotifyNewKeyFrame(); // fails if state is not RUNNING

    //Check current Thread state
    bool isStopped();
    bool isFinished();
    bool isRunning();
    bool isIdle(); //waiting on condition variable
    bool isStopRequested(); //true if stopped or stop requested

    int KeyframesInQueue(){
        lock_guard<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

  void AddHookNewKeyFrame(std::function<void(long unsigned int)> hook){
        lock_guard<mutex> lock(mMutexNewKFs);
        mlNewKFHooks.push_back(hook);
   }

private:
    enum class LocalMappingState
    {
        RUNNING,
        STOPPED,
        FINISHED,
    };

    void ResetIfRequested();
    LocalMappingState UpdateState();

    bool GetNewKeyFrame();
    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

private:
    Map* mpMap;
    LoopClosing* mpLoopCloser;
    bool mbMonocular;

    std::atomic<LocalMappingState> mState;
    std::atomic_bool mbIdle; //if true then LocalMapping thread probably waiting on CV

    //state signals
    std::atomic_bool mbFinishRequested;
    std::atomic_int mNStopRequested; //counts stop-release

    //other signals
    std::atomic_bool mbResetRequested;
    bool mbAbortBA; //TODO clean this (cf LoopClossing and g2o)
    bool mbNotifyNewKF;

    std::list<KeyFrame*> mlNewKeyFrames;
    std::mutex mMutexNewKFs;
    std::condition_variable mCvNewKFs;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::list<std::function<void(long unsigned int)>> mlNewKFHooks;

    string mLinearSolverName;
    string mOptimizationAlgorithmName;

};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
