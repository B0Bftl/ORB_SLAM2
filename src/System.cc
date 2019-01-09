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



#include "System.h"
#include "Converter.h"
#include "Optimizer.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <time.h>



static bool has_suffix(const std::string &str, const std::string &suffix)
{
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace ORB_SLAM2
{

bool has_suffix(const std::string &str, const std::string &suffix) {
  std::size_t index = str.find(suffix, str.size() - suffix.size());
  return (index != std::string::npos);
}

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor)
    : mSensor(sensor), mpViewer(nullptr), mbReset(false), mbActivateLocalizationMode(false),
      mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    cv::FileNode mapfilen = fsSettings["Map.mapfile"];
    bool bReuseMap = false;
    if (!mapfilen.empty())
    {
        mapfile = (string)mapfilen;
    }

    bool loadBinaryMap;
    bool loadTextMap;
    string loadMap = fsSettings["Map.loadMap"];
    loadBinaryMap = loadMap == "bin";
    loadTextMap = loadMap == "txt";



    string saveBin = fsSettings["Map.saveAsBin"];
    save_map_bin = (saveBin == "true") && !mapfilen.empty();

    string saveText = fsSettings["Map.saveAsBin"];
    save_map_text = (saveText == "true") && !mapfilen.empty();


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
    clock_t tStart = clock();
    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else if(has_suffix(strVocFile, ".bin"))
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    else
        bVocLoad = false;
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    printf("Vocabulary loaded in %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(mpVocabulary);

    //Create KeyFrame Database
    //Create the Map
    bReuseMap = false;
    if (!mapfile.empty())
    {
        if (loadBinaryMap)
            bReuseMap = LoadMapBin(mapfile);
        else if(loadTextMap)
            bReuseMap = LoadMapText(mapfile);
    }

    if(!bReuseMap)
    {
        mpKeyFrameDatabase = new KeyFrameDatabase(mpVocabulary);
        mpMap = new Map();
    }

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpMap, mpKeyFrameDatabase,
                             strSettingsFile, mSensor, bReuseMap);


	string optimizationAlgorithmName = (string) fsSettings["Optimizer.Algorithm"];
	string linearSolverName = (string) fsSettings["Optimizer.linearSolver"];

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR, linearSolverName, optimizationAlgorithmName);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    string useViewer = fsSettings["Viewer.useViewer"];
    if(useViewer == "true")
    {
        //Create Drawers. These are used by the Viewer
        mpFrameDrawer = new FrameDrawer(mpMap);
        mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

        //Crate the viewer and start viewer thread
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,strSettingsFile, bReuseMap);
        mptViewer = new thread(&Viewer::Run, mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    CheckState();

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    UpdateTrackingState(Tcw);

    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    CheckState();

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    UpdateTrackingState(Tcw);

    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    CheckState();

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    UpdateTrackingState(Tcw);

    return Tcw;
}

void System::CheckState()
{
    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->RequestRelease();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        Reset();
        mbReset = false;
    }
    }
}

void System::UpdateTrackingState(const cv::Mat& Tcw)
{
    {
    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    }

    //Update frame drawer and current camera pose in map drawer
    if(mpViewer)
    {
        mpFrameDrawer->Update(mpTracker);
        mpMapDrawer->SetCurrentCameraPose(Tcw);
    }
    if (mTrackingState == Tracking::LOST) cout << "Track Lost!" << endl;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

Map* System::GetMap()
{
    return mpMap;
}

ORBVocabulary* System::GetVocabulary()
{
    return mpVocabulary;
}


void System::StartGlobalBundleAdjustment()
{
    // Get Map Mutex
    if(mpLoopCloser->isRunningGBA()) return;    
    mpLoopCloser->StopGlobalBundleAdjustment();
    mpLoopCloser->StartGlobalBundleAdjustment();
}

bool System::isIdle()
{
  return mpLocalMapper->isIdle() && mpLoopCloser->isIdle() && !mpLoopCloser->isRunningGBA();
}
  
void System::AskReset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Reset()
{
    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...\n";
    mpLocalMapper->RequestReset();
    cout << "Reseting Local Mapper: DONE\n";

    // Reset Loop Closing
    cout << "Reseting Loop Closing...\n";
    mpLoopCloser->RequestReset();
    cout << "Reseting Loop Closing: DONE\n";

    // Clear BoW Database
    cout << "Reseting Database...\n";
    mpKeyFrameDatabase->clear();
    cout << "Reseting Database: DONE\n";

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    //Reset F/KF ids
    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;

    mpTracker->Reset();

    if(mpViewer)
        mpViewer->Release();
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");

	Optimizer::printTime();

	// Save map as bin/text
    if (save_map_bin)
        SaveMapBin(mapfile);

    if (save_map_text)
        SaveMapText(mapfile);
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

template<>
void System::AddHook<System::HookType::NewKF>(std::function<void(long unsigned int)> hook)
{
  mpLocalMapper->AddHookNewKeyFrame(hook);
}

void System::SaveMapBin(const string &filename)
{
    string file = filename + ".bin";
    std::ofstream out(file, std::ios_base::binary);
    if (!out)
    {
        cerr << "Cannot Write to Mapfile: " << file << std::endl;
        exit(-1);
    }
    cout << "Saving Mapfile: " << file << std::flush;
    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
    oa << mpMap;
    oa << mpKeyFrameDatabase;
    cout << " ...done" << std::endl;
    out.close();
}

void System::SaveMapText(const string &filename)
{
    string file = filename + ".txt";
    std::ofstream out(file, std::ios_base::trunc);

    if (!out)
    {
        cerr << "Cannot Write to Mapfile: " << file << std::endl;
        exit(-1);
    }
    cout << "Saving Mapfile as Text: " << file << std::flush;
    boost::archive::text_oarchive oa (out);
    oa << mpMap;
    oa << mpKeyFrameDatabase;
    cout << " ...done" << std::endl;
    out.close();

}

bool System::LoadMapText(const string &filename)
{
    string file = filename + ".txt";
    std::ifstream in(file);
    if (!in)
    {
        cerr << "Cannot Open Text Mapfile: " << file << " , You need create it first!" << std::endl;
        return false;
    }
    cout << "Loading Text Mapfile: " << file << std::flush;
    boost::archive::text_iarchive ia(in);
    cout << "1";
    ia >> mpMap;
	cout << "2";
    ia >> mpKeyFrameDatabase;
	cout << "3";
    mpKeyFrameDatabase->SetORBvocabulary(mpVocabulary);
    cout << " ...done" << std::endl;
    cout << "Map Reconstructing" << flush;
    vector<ORB_SLAM2::KeyFrame*> vpKFS = mpMap->GetAllKeyFrames();
    unsigned long mnFrameId = 0;
    for (auto it:vpKFS) {
        it->SetORBvocabulary(mpVocabulary);
        it->ComputeBoW();
        if (it->mnFrameId > mnFrameId)
            mnFrameId = it->mnFrameId;
    }
    Frame::nNextId = mnFrameId;
    cout << " ...done" << endl;
    in.close();
    return true;
}

bool System::LoadMapBin(const string &filename)
{
    string file = filename + ".bin";
    std::ifstream in(file, std::ios_base::binary);
    if (!in)
    {
        cerr << "Cannot Open Mapfile: " << file << " , You need create it first!" << std::endl;
        return false;
    }
    cout << "Loading Mapfile: " << file << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> mpMap;
    ia >> mpKeyFrameDatabase;
    mpKeyFrameDatabase->SetORBvocabulary(mpVocabulary);
    cout << " ...done" << std::endl;
    cout << "Map Reconstructing" << flush;
    vector<ORB_SLAM2::KeyFrame*> vpKFS = mpMap->GetAllKeyFrames();
    unsigned long mnFrameId = 0;
    for (auto it:vpKFS) {
        it->SetORBvocabulary(mpVocabulary);
        it->ComputeBoW();
        if (it->mnFrameId > mnFrameId)
            mnFrameId = it->mnFrameId;
    }
    Frame::nNextId = mnFrameId;
    cout << " ...done" << endl;
    in.close();
    return true;
}

} //namespace ORB_SLAM
