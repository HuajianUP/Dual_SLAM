/**
* Take Tracking.h of ORB-SLAM2 as reference.
*
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
*/

#ifndef RECOVERY_H
#define RECOVERY_H


#include "Frame.h"

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "Viewer.h"

#include "FrameDrawer.h"
#include "Map.h"

#include "Tracking.h"
#include <opencv2/core/core.hpp>
#include <mutex>
#include "ORBVocabulary.h"

#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"


using namespace ORB_SLAM2;
namespace Dual_SLAM
{
    
class Recovery
{
public:
    static bool recoveringFlag;
    enum eRecoveryState{
        SYSTEM_FAIL=-1,
        NOT_INITIALIZED=0,
        OK=1,
        RECOVERING=2,
        SIM3_READY=3,
        ICP_READY=4,
        SYSTEM_READY=5
    };
    eRecoveryState mState;

    Recovery();
    Recovery(list<Frame> & rFrames, vector<KeyFrame*> rLocalKeyFramesWhenBreak, vector<MapPoint*> rLocalMapPointsWhenBreak, vector<KeyFrame*> rLocalKeyFramesAfterInit, const long unsigned int rframeSNWhenBreak,
        Map* mpMap, LocalMapping *mapMapper, LoopClosing* LoopCloser, KeyFrameDatabase* rKFDB);
    Recovery(list<Frame> & rFrames, KeyFrame* rKFcur , KeyFrame* rKFini, Frame & rLastFrame, const long unsigned int rframeSNWhenBreak, 
        Map*mpMap, LocalMapping *mapMapper, LoopClosing* LoopCloser, KeyFrameDatabase* rKFDB);
    void Run();


    void RecoverInitialMapPoints();
    void ChangeInitialMapPoints();

    void SearchLocalPoints();
    bool TrackLocalMap();

    bool NeedNewKeyFrame();

    void CreateNewKeyFrame();
    void ProcessNewKeyFrame();
    void MapPointCulling();
    void CreateNewMapPoints();
    void SearchInNeighbors();
    cv::Mat ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);
    void UpdateLocalKeyFrames();
    void UpdateLocalPoints();


    bool DetectInOldMap(g2o::Sim3 & mg2oScw);
    void RecoveryMapBySim3(g2o::Sim3 & mg2oScw);
    void RecoveryMapByICP();
    

protected:
    Frame mCurrentFrame; 
    Frame mLastFrame;

    KeyFrame* mpCurrentKeyFrame;
    KeyFrame* mpLastKeyFrame;
    unsigned int mnLastKeyFrameId;

    //ComputeSim3
    vector<MapPoint*> mvpLoopMapPoints;
    vector<MapPoint*> mvpCurrentMatchedPoints;
    KeyFrame* mpMatchedKF;

//private:
    Frame mRLastFrame;
    KeyFrame* mRKFcur;
    KeyFrame* mRKFini;

    Map* mpMap;
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopCloser;
    KeyFrameDatabase* mpKeyFrameDB;

    list<Frame> framesNeedRecovery;
    vector<KeyFrame*> mvRLocalKeyFrames;
    vector<MapPoint*> mvRLocalMapPoints;
    vector<KeyFrame*> mvRLocalKeyFramesWhenBreak;
    vector<MapPoint*> mvRLocalMapPointsWhenBreak;
    long unsigned int mRframeSNWhenBreak;


    //mapping
    vector<MapPoint*> mvInitialMapPoints;
    vector<MapPoint*> mvInitialMapPointsChanged;
    KeyFrame* mInitialKeyFrame;

    list<MapPoint*> mlpRecentAddedMapPoints;
    bool mbAbortBA;
    //map<MapPoint*,size_t> mObservations;
    
};

}

/*
namespace Dual_SLAM
{

class Recovery
{  

protected:
    std::thread* repairMapping;
    //computer sim3
    g2o::Sim3 mg2oScw;
    vector<MapPoint*> mvpLoopMapPoints;
    vector<MapPoint*> mvpCurrentMatchedPoints;
    bool ComputeSim3(KeyFrame* mpCurrentKF,KeyFrame* MatchedKF,vector<MapPoint*> vvpMapPointMatches);
    void CorrectLoop(KeyFrame* mpCurrentKF,KeyFrame* mpMatchedKF);
    bool mbMonocular;
    bool mbAbortBA;

    KeyFrame* mpCurrentKeyFrame;
    KeyFrame* rOldMapKFcur;
    KeyFrame* rOldMapKFini;

    std::list<MapPoint*> mlpRecentAddedMapPoints;
    std::list<KeyFrame*> mlNewKeyFrames;
    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    void ProcessNewKeyFrame();
    void MapPointCulling();
    void CreateNewMapPoints();
    void SearchInNeighbors();
    cv::Mat ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);
    void KeyFrameCulling();
    void InsertKeyFrame(KeyFrame *pKF);
    bool CheckNewKeyFrames();
    void buildMap();
    

public:
    static unsigned int nRepairStop;
    bool nfirstRepair;
    void static PnPGetRT(Frame & pF1, Frame & pF2, const vector<cv::DMatch>& m_InlierMatches);
    void static matchSURF(Frame & pF1, Frame & pF2, vector<cv::DMatch>& m_InlierMatches);
    void static CreateNewSiftMapPoints(Frame & pF1, Frame & pF2, const vector<cv::DMatch>& m_InlierMatches);
    void static CreateNewSiftMapPoints(KeyFrame* pKF1, KeyFrame* pKF2);


	Recovery(list<Frame> & rFrames, Map*pMap , Map*rMap, KeyFrameDatabase* rKFDB, LoopClosing* LoopCloser, LocalMapping *mapMapper,
     KeyFrame* rKFcur , KeyFrame* rKFini, Frame & rLastFrame , Frame & riniFrame, const int rMaxFrames,const int sensor);
	// Main tracking function. 
	void Run();
    void InterruptBA();
    
public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;

    // Current Frame
    Frame mCurrentFrame;

	// Input sensor
	int mSensor;

	bool mbOnlyTracking;
    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;


protected:
	void repairTrack();
    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;


    list<Frame> mrepairFrames;
    
    //Map
    Map* mpMap;
    Map* moldMap;

    //BoW
    KeyFrameDatabase* mpKeyFrameDB;
    LoopClosing* mpLoopCloser;
    LocalMapping *mpMapper;


    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;


    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    Frame mLLastFrame;
    Frame miniFrame;

    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    list<MapPoint*> mlpTemporalPoints;
};

} //namespace Dual_SLAM
*/
#endif 