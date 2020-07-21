/**
* some codes are references to ORB-SLAM2.
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
*/
#include "Recovery.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "PnPsolver.h"
#include "Sim3Solver.h"
#include "Converter.h"
#include <unistd.h>

namespace Dual_SLAM
{

	bool Recovery::recoveringFlag = true;
	Recovery::Recovery(list<Frame> & rFrames, vector<KeyFrame*> rLocalKeyFramesWhenBreak,vector<MapPoint*> rLocalMapPointsWhenBreak, vector<KeyFrame*> rLocalKeyFramesAfterInit, const long unsigned int rframeSNWhenBreak,
					Map* mpMap, LocalMapping *mapMapper, LoopClosing* LoopCloser, KeyFrameDatabase* rKFDB):framesNeedRecovery(rFrames), 
					mvRLocalKeyFramesWhenBreak(rLocalKeyFramesWhenBreak), mvRLocalMapPointsWhenBreak(rLocalMapPointsWhenBreak), mvRLocalKeyFrames(rLocalKeyFramesAfterInit),
					mRframeSNWhenBreak(rframeSNWhenBreak), mpMap(mpMap), mpLocalMapper(mapMapper), mpLoopCloser(LoopCloser), mpKeyFrameDB(rKFDB),mState(NOT_INITIALIZED)
	{
		mpLoopCloser->RequestStop();
		mbAbortBA = false;
	}
	Recovery::Recovery(list<Frame> & rFrames, KeyFrame* rKFcur , KeyFrame* rKFini, Frame & rLastFrame, const long unsigned int rframeSNWhenBreak, 
				Map*mpMap, LocalMapping *mapMapper, LoopClosing* LoopCloser, KeyFrameDatabase* rKFDB):framesNeedRecovery(rFrames), mRKFcur(rKFcur), mRKFini(rKFini), 
				mRLastFrame(rLastFrame), mRframeSNWhenBreak(rframeSNWhenBreak), mpMap(mpMap), mpLocalMapper(mapMapper), mpLoopCloser(LoopCloser), 
				mpKeyFrameDB(rKFDB),mState(NOT_INITIALIZED)
	{
		mpLoopCloser->RequestStop();
	}

	void Recovery::Run()
	{
	#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
	#endif

		//cout<<mRKeyFrameIdWhenBreak<<" "<<mRKFcur->FrameSN<<" "<<mRKFcur->mnId<<" "<<mRKFini->FrameSN<<" "<<mRKFini->mnId<<endl;
		mCurrentFrame = framesNeedRecovery.back();
		mLastFrame = mCurrentFrame;
		mpLastKeyFrame = mvRLocalKeyFrames.back();

		UpdateLocalPoints();
		g2o::Sim3 mg2oScw;
		cout<<"recovery thread starts: "<<mpMap->KeyFramesInMap()<<" the number of frames Need Recovery: "<<framesNeedRecovery.size()<<"mvRLocalMapPointsWhenBreak: "<<mvRLocalMapPointsWhenBreak.size()<<endl;
		cout<<"the number of local KeyFrames:"<<mvRLocalKeyFrames.size()<<" last KeyFrame ID:"<< mpLastKeyFrame->mnId <<" SN:"<< mpLastKeyFrame->FrameSN <<endl;
		cout<<"the number of local MapPoint:"<<mvRLocalMapPoints.size()<<endl;
		cout<<"KeyFrameId When Break:"<<mRframeSNWhenBreak<<endl;
		for(list<Frame>::reverse_iterator it = framesNeedRecovery.rbegin(); it != framesNeedRecovery.rend(); ++it)
        {
        	mCurrentFrame = *it;
            //cout<<mCurrentFrame.FrameSN<<" "<<endl;
            if(NeedNewKeyFrame())
            {
			    mpCurrentKeyFrame = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);
			    //KeyFrame::nNextId--;
			    //mpCurrentKeyFrame->mnId = mpLastKeyFrame->mnId - 1;
            	//cout<<"Create New KeyFrame... ";
            	CreateNewKeyFrame();
            	mpLastKeyFrame = mpCurrentKeyFrame;

				if(mState == OK && mCurrentFrame.FrameSN < mRframeSNWhenBreak + 5)
				{					
					mState = RECOVERING;
				}
				if(mState == RECOVERING)
				{
					cout<<"Trying to recover ...... "<<endl;
					if(DetectInOldMap(mg2oScw))
					{				
						mState = SIM3_READY;
						break;	
					}
				}
            }
        }
        //cout<<"END, RecoverInitialMapPoints......."<<endl;

    	mpLocalMapper->RequestStop();
	    // Wait until Local Mapping has effectively stopped
	    while(!mpLocalMapper->isStopped())
	    {
	        usleep(1000);
	    }
	    cout<<" recoveriiiiiiiiiiiiiiiiiiiiiiiiiiing   "<<endl;
		RecoverInitialMapPoints();
		if(mState == SIM3_READY)
		{
			RecoveryMapBySim3(mg2oScw);
		}
            
		mpLocalMapper->Release();  
        mpLoopCloser->ReleaseStop();

    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
	#endif
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        cout<<" Done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!:   "<<endl;
        cout<<"Time: "<<ttrack<<endl;
	}


	bool Recovery::NeedNewKeyFrame()
	{
		ORBmatcher matcher(0.9,true);
        vector<MapPoint*> vpMapPointMatches;
        int inliers = 0;
        int nmatches = matcher.SearchByGMS(mpLastKeyFrame,mCurrentFrame,vpMapPointMatches, inliers);
        mCurrentFrame.mvpMapPoints = vpMapPointMatches;

        cout<<"CurrentFrame: "<<mCurrentFrame.mnId<< "   mpLastKeyFrame: "<<mpLastKeyFrame->FrameSN;
        cout<<" the nmatches are "<< nmatches << " inliers are " << inliers <<endl; //"Key size:"<<mCurrentFrame.N<<" "<<mpLastKeyFrame->N<<

        int th = 70;
        if(mState == NOT_INITIALIZED)
        	th = 4*th;
        if(nmatches < 300 || inliers < th)
        {
        	if(inliers < 10)
        	{
        		//return false;
        		/*
        		nmatches = matcher.SearchByGMS(mpLastKeyFrame,mLastFrame,vpMapPointMatches, inliers);
			    for(int i =0; i < mLastFrame.N; i++)
			    {
			        if(mLastFrame.mvpMapPoints[i] && !vpMapPointMatches[i])
			        {
			        	vpMapPointMatches[i] = mLastFrame.mvpMapPoints[i];
			        	inliers++;
			        }
			        if(!mLastFrame.mvpMapPoints[i] && vpMapPointMatches[i])
			        {
			        	mLastFrame.mvpMapPoints[i] = vpMapPointMatches[i];
			        }
			    }*/
			    //if(inliers < 20)
			    //	return false;

        		Frame temp = mCurrentFrame;
				mCurrentFrame = mLastFrame;
				vpMapPointMatches = mLastFrame.mvpMapPoints;
				mLastFrame = temp;
			}
			else
			{
				mLastFrame = mCurrentFrame;
			}

			PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vpMapPointMatches);
	        pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
	        // Perform 5 Ransac Iterations
	        vector<bool> vbInliers;
	        int PnPInliers;
	        bool bNoMore;
	        cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,PnPInliers);
	        if(!Tcw.empty())
				mCurrentFrame.SetPose(Tcw);
			else
			{
				cout<<"Noooooooooooooooooooooooooooooooo............"<<endl;
				mCurrentFrame.SetPose(mpLastKeyFrame->GetPose());
			}

			mCurrentFrame.ComputeBoW();

	        Optimizer::PoseOptimization(& mCurrentFrame); 
	        int nmatchesMap = 0;
		    for(int i =0; i < mCurrentFrame.N; i++)
		    {
		        if(mCurrentFrame.mvpMapPoints[i])
		        {
		            if(mCurrentFrame.mvbOutlier[i])
		            {
		                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
		                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
		                mCurrentFrame.mvbOutlier[i] = false;
		                //pMP->mbTrackInView = false;
	        			//pMP->mnLastFrameSeen = mCurrentFrame.mnId;
		                inliers--;
		            }
		            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
		                nmatchesMap++;
		        }
		    }
		    //cout<<" after Optimize, inliers are "<<inliers<<" "<<nmatchesMap<<endl;
		    if(inliers > 10 && TrackLocalMap())
		    {
			    return true;
		    }     
        }
        return false;
	}

	bool Recovery::TrackLocalMap()
	{

	    SearchLocalPoints();
	    // Optimize Pose
   
	    Optimizer::PoseOptimization(&mCurrentFrame);
	    int mnMatchesInliers = 0;

	    // Update MapPoints Statistics
	    for(int i=0; i<mCurrentFrame.N; i++)
	    {
	        if(mCurrentFrame.mvpMapPoints[i])
	        {
	            if(!mCurrentFrame.mvbOutlier[i])
	            {
	                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
	            }
	        }
	    }

	    if(mnMatchesInliers<20)
	    {
	        cout<<"TrackLocalMap "<<mnMatchesInliers<<endl;
	        return false;
	    }
	    else
	        return true;
	}

	void Recovery::SearchLocalPoints()
	{
	    // Do not search map points already matched
	    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
	    {
	        MapPoint* pMP = *vit;
	        if(pMP)
	        {
	            if(pMP->isBad())
	            {
	                *vit = static_cast<MapPoint*>(NULL);
	            }
	            else
	            {
	                pMP->IncreaseVisible();
	                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
	                pMP->mbTrackInView = false;
	            }
	        }
	    }

	    int nToMatch=0;

	    // Project points in frame and check its visibility
	    for(vector<MapPoint*>::iterator vit=mvRLocalMapPoints.begin(), vend = mvRLocalMapPoints.end(); vit!=vend; vit++)
	    {
	        MapPoint* pMP = *vit;
	        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
	            continue;
	        if(pMP->isBad())
	            continue;
	        // Project (this fills MapPoint variables for matching)
	        if(mCurrentFrame.isInFrustum(pMP,0.5))
	        {
	            pMP->IncreaseVisible();
	            nToMatch++;
	        }
	    }

	    if(nToMatch>0)
	    {
	        ORBmatcher matcher(0.8);
	        matcher.SearchByProjection(mCurrentFrame,mvRLocalMapPoints,2);
	    }
	}

	void Recovery::ChangeInitialMapPoints()
	{
		mvInitialMapPoints = mCurrentFrame.mvpMapPoints;

		mInitialKeyFrame = mpCurrentKeyFrame;
		for(int i =0; i < mCurrentFrame.N; i++)
	    {
	    	MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
	        if(pMP && !pMP->isBad())
	        {
	            
			    cv::Mat worldPos = pMP->GetWorldPos();			  
		        MapPoint* pMPNew = new MapPoint(worldPos, mpCurrentKeyFrame, mpMap);

		        mpCurrentKeyFrame->AddMapPoint(pMPNew,i);

		        pMPNew->AddObservation(mpCurrentKeyFrame,i);
		        pMPNew->ComputeDistinctiveDescriptors();
		        pMPNew->UpdateNormalAndDepth();

		        mCurrentFrame.mvpMapPoints[i] = pMPNew;

		        /*
		        pKFini->AddMapPoint(pMPNew,i);
		        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

		        pMP->AddObservation(pKFini,i);
		        pMP->AddObservation(pKFcur,mvIniMatches[i]);

		        pMP->ComputeDistinctiveDescriptors();
		        pMP->UpdateNormalAndDepth();

		        //Fill Current Frame structure
		        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
				*/
	        }
	    }
	    mvInitialMapPointsChanged = mCurrentFrame.mvpMapPoints;

	}

	void Recovery::RecoverInitialMapPoints() // connect the map
	{
		for(int i =0; i < mvInitialMapPoints.size(); i++)
	    {
	    	MapPoint* pMP = mvInitialMapPoints[i];
	    	MapPoint* pCurMP = mvInitialMapPointsChanged[i];
            if(pCurMP)
                pCurMP->Replace(pMP);
	    }

	    mInitialKeyFrame-> UpdateConnections();
	    const vector<KeyFrame*> vpNeighKFs = mInitialKeyFrame->GetBestCovisibilityKeyFrames(10);
	    for(size_t i=0; i<vpNeighKFs.size(); i++)
	    {
	    	vpNeighKFs[i]->UpdateConnections();
	    }
	    mInitialKeyFrame ->SetBadFlag();
	}

	void Recovery::CreateNewKeyFrame()
	{
		//cout<<"ProcessNewKeyFrame..."<<endl;
		if(mState == NOT_INITIALIZED)
	    {
	    	cout<<"ChangeInitialMapPoints............."<<endl;
	    	ChangeInitialMapPoints();
	    	mState = OK;
	    }
	    else
			ProcessNewKeyFrame();
		//cout<<"MapPointCulling..."<<endl;
		MapPointCulling();
		//cout<<"CreateNewMapPoints..."<<endl;
		CreateNewMapPoints();
		SearchInNeighbors();
		Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap);

		//Update LocalMap
		mpKeyFrameDB->add(mpCurrentKeyFrame);
    	UpdateLocalKeyFrames();
		UpdateLocalPoints();
	}

	void Recovery::ProcessNewKeyFrame()
	{
	    // Compute Bags of Words structures
	    mpCurrentKeyFrame->ComputeBoW();

	    // Associate MapPoints to the new keyframe and update normal and descriptor
	    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
	    //cout<<"  Associate MapPoints to the new keyframe"<<endl;
	    for(size_t i=0; i<vpMapPointMatches.size(); i++)
	    {
	        MapPoint* pMP = vpMapPointMatches[i];
	        if(pMP)//pMP
	        {
	            if(!pMP->isBad())
	            {
	                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
	                {
	                	//cout<<"AddObservation"<<endl;
	                    pMP->AddObservation(mpCurrentKeyFrame, i);
	                    //cout<<"UpdateNormalAndDepth"<<endl;
	                    pMP->UpdateNormalAndDepth();
	                    //cout<<"ComputeDistinctiveDescriptors"<<endl;
	                    pMP->ComputeDistinctiveDescriptors();
	                }
	                else // this can only happen for new stereo points inserted by the Tracking
	                {
	                    mlpRecentAddedMapPoints.push_back(pMP);
	                }
	            }
	        }
	    }    
	    //cout<<"Update links in the Covisibility Graph"<<endl;
	    // Update links in the Covisibility Graph
	    mpCurrentKeyFrame->UpdateConnections();

	    // Insert Keyframe in Map
	    mpMap->AddKeyFrame(mpCurrentKeyFrame);
	}

	void Recovery::MapPointCulling()
	{
	    // Check Recent Added MapPoints
	    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
	    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

	    int nThObs = 2;
	    const int cnThObs = nThObs;

	    while(lit!=mlpRecentAddedMapPoints.end())
	    {
	        MapPoint* pMP = *lit;
	        if(pMP->isBad())
	        {
	            lit = mlpRecentAddedMapPoints.erase(lit);
	        }
	        else if(pMP->GetFoundRatio()<0.25f )
	        {
	            pMP->SetBadFlag();
	            lit = mlpRecentAddedMapPoints.erase(lit);
	        }
	        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
	        {
	            pMP->SetBadFlag();
	            lit = mlpRecentAddedMapPoints.erase(lit);
	        }
	        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
	            lit = mlpRecentAddedMapPoints.erase(lit);
	        else
	            lit++;
	    }
	}

	void Recovery::CreateNewMapPoints()
	{
	    // Retrieve neighbor keyframes in covisibility graph
	    int nn = 20;

	    vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

	    //if(vpNeighKFs.empty())
	    //	vpNeighKFs.push_back(mpLastKeyFrame);

	    ORBmatcher matcher(0.6,false);

	    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
	    cv::Mat Rwc1 = Rcw1.t();
	    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
	    cv::Mat Tcw1(3,4,CV_32F);
	    Rcw1.copyTo(Tcw1.colRange(0,3));
	    tcw1.copyTo(Tcw1.col(3));
	    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

	    const float &fx1 = mpCurrentKeyFrame->fx;
	    const float &fy1 = mpCurrentKeyFrame->fy;
	    const float &cx1 = mpCurrentKeyFrame->cx;
	    const float &cy1 = mpCurrentKeyFrame->cy;
	    const float &invfx1 = mpCurrentKeyFrame->invfx;
	    const float &invfy1 = mpCurrentKeyFrame->invfy;

	    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

	    int nnew=0;

	    // Search matches with epipolar restriction and triangulate
	    for(size_t i=0; i<vpNeighKFs.size(); i++)
	    {
	    	//cout<<"CreateNewMapPoints "<< i<<endl;
	        KeyFrame* pKF2 = vpNeighKFs[i];

	        if(!pKF2 || pKF2->isBad())
	        {
	        	cout<<"warning................................................................"<<endl;
	        	continue;
	        }
	        // Check first that baseline is not too short
	        cv::Mat Ow2 = pKF2->GetCameraCenter();
	        cv::Mat vBaseline = Ow2-Ow1;
	        const float baseline = cv::norm(vBaseline);

            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
			//cout<<"start ComputeF12 "<< i<<endl;
	        // Compute Fundamental Matrix
	        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);
			//cout<<"ComputeF12 "<< i<<endl;
	        // Search matches that fullfil epipolar constraint
	        vector<pair<size_t,size_t> > vMatchedIndices;
	        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);
	        
	        cv::Mat Rcw2 = pKF2->GetRotation();
	        cv::Mat Rwc2 = Rcw2.t();
	        cv::Mat tcw2 = pKF2->GetTranslation();
	        cv::Mat Tcw2(3,4,CV_32F);
	        Rcw2.copyTo(Tcw2.colRange(0,3));
	        tcw2.copyTo(Tcw2.col(3));

	        const float &fx2 = pKF2->fx;
	        const float &fy2 = pKF2->fy;
	        const float &cx2 = pKF2->cx;
	        const float &cy2 = pKF2->cy;
	        const float &invfx2 = pKF2->invfx;
	        const float &invfy2 = pKF2->invfy;
			//cout<<"Triangulate each match "<< i<<endl;
	        // Triangulate each match
	        const int nmatches = vMatchedIndices.size();
	        for(int ikp=0; ikp<nmatches; ikp++)
	        {
	            const int &idx1 = vMatchedIndices[ikp].first;
	            const int &idx2 = vMatchedIndices[ikp].second;

	            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
	            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
	            bool bStereo1 = kp1_ur>=0;

	            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
	            const float kp2_ur = pKF2->mvuRight[idx2];
	            bool bStereo2 = kp2_ur>=0;

	            // Check parallax between rays
	            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
	            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

	            cv::Mat ray1 = Rwc1*xn1;
	            cv::Mat ray2 = Rwc2*xn2;
	            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

	            float cosParallaxStereo = cosParallaxRays+1;
	            float cosParallaxStereo1 = cosParallaxStereo;
	            float cosParallaxStereo2 = cosParallaxStereo;

	            if(bStereo1)
	                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
	            else if(bStereo2)
	                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

	            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

	            cv::Mat x3D;
	            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
	            {
	                // Linear Triangulation Method
	                cv::Mat A(4,4,CV_32F);
	                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
	                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
	                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
	                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

	                cv::Mat w,u,vt;
	                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

	                x3D = vt.row(3).t();

	                if(x3D.at<float>(3)==0)
	                    continue;

	                // Euclidean coordinates
	                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

	            }
	            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
	            {
	                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);                
	            }
	            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
	            {
	                x3D = pKF2->UnprojectStereo(idx2);
	            }
	            else
	                continue; //No stereo and very low parallax

	            cv::Mat x3Dt = x3D.t();

	            //Check triangulation in front of cameras
	            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
	            if(z1<=0)
	                continue;

	            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
	            if(z2<=0)
	                continue;

	            //Check reprojection error in first keyframe
	            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
	            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
	            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
	            const float invz1 = 1.0/z1;

	            if(!bStereo1)
	            {
	                float u1 = fx1*x1*invz1+cx1;
	                float v1 = fy1*y1*invz1+cy1;
	                float errX1 = u1 - kp1.pt.x;
	                float errY1 = v1 - kp1.pt.y;
	                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
	                    continue;
	            }
	            else
	            {
	                float u1 = fx1*x1*invz1+cx1;
	                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
	                float v1 = fy1*y1*invz1+cy1;
	                float errX1 = u1 - kp1.pt.x;
	                float errY1 = v1 - kp1.pt.y;
	                float errX1_r = u1_r - kp1_ur;
	                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
	                    continue;
	            }

	            //Check reprojection error in second keyframe
	            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
	            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
	            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
	            const float invz2 = 1.0/z2;
	            if(!bStereo2)
	            {
	                float u2 = fx2*x2*invz2+cx2;
	                float v2 = fy2*y2*invz2+cy2;
	                float errX2 = u2 - kp2.pt.x;
	                float errY2 = v2 - kp2.pt.y;
	                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
	                    continue;
	            }
	            else
	            {
	                float u2 = fx2*x2*invz2+cx2;
	                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
	                float v2 = fy2*y2*invz2+cy2;
	                float errX2 = u2 - kp2.pt.x;
	                float errY2 = v2 - kp2.pt.y;
	                float errX2_r = u2_r - kp2_ur;
	                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
	                    continue;
	            }

	            //Check scale consistency
	            cv::Mat normal1 = x3D-Ow1;
	            float dist1 = cv::norm(normal1);

	            cv::Mat normal2 = x3D-Ow2;
	            float dist2 = cv::norm(normal2);

	            if(dist1==0 || dist2==0)
	                continue;

	            const float ratioDist = dist2/dist1;
	            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

	            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
	                continue;*/
	            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
	                continue;
	            //cout<<"Triangulation is succesfull"<<endl;
	            // Triangulation is succesfull
	            MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);

	            pMP->AddObservation(mpCurrentKeyFrame,idx1);            
	            pMP->AddObservation(pKF2,idx2);

	            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
	            pKF2->AddMapPoint(pMP,idx2);

	            pMP->ComputeDistinctiveDescriptors();

	            pMP->UpdateNormalAndDepth();

	            mpMap->AddMapPoint(pMP);
	            mlpRecentAddedMapPoints.push_back(pMP);

	            nnew++;
	        }
	    }
	    cout<<"Finish CreateNewMapPoints "<<nnew<<endl;
	}

	cv::Mat Recovery::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
	{
	    cv::Mat R1w = pKF1->GetRotation();
	    cv::Mat t1w = pKF1->GetTranslation();
	    cv::Mat R2w = pKF2->GetRotation();
	    cv::Mat t2w = pKF2->GetTranslation();

	    cv::Mat R12 = R1w*R2w.t();
	    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

	    //cv::Mat t12x = SkewSymmetricMatrix(t12);

		cv::Mat t12x = (cv::Mat_<float>(3,3) <<   0, -t12.at<float>(2), t12.at<float>(1),
            t12.at<float>(2),     0,-t12.at<float>(0),
            -t12.at<float>(1),  t12.at<float>(0),  0);

	    const cv::Mat &K1 = pKF1->mK;
	    const cv::Mat &K2 = pKF2->mK;

	    return K1.t().inv()*t12x*R12*K2.inv();
	}

	void Recovery::SearchInNeighbors()
	{
	    // Retrieve neighbor keyframes
	    int nn = 20;

	    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
	    vector<KeyFrame*> vpTargetKFs;
	    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
	    {
	        KeyFrame* pKFi = *vit;
	        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
	            continue;
	        vpTargetKFs.push_back(pKFi);
	        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

	        // Extend to some second neighbors
	        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
	        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
	        {
	            KeyFrame* pKFi2 = *vit2;
	            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
	                continue;
	            vpTargetKFs.push_back(pKFi2);
	        }
	    }


	    // Search matches by projection from current KF in target KFs
	    ORBmatcher matcher;
	    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
	    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
	    {
	        KeyFrame* pKFi = *vit;

	        matcher.Fuse(pKFi,vpMapPointMatches);
	    }

	    // Search matches by projection from target KFs in current KF
	    vector<MapPoint*> vpFuseCandidates;
	    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

	    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
	    {
	        KeyFrame* pKFi = *vitKF;

	        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

	        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
	        {
	            MapPoint* pMP = *vitMP;
	            if(!pMP)
	                continue;
	            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
	                continue;
	            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
	            vpFuseCandidates.push_back(pMP);
	        }
	    }

	    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);


	    // Update points
	    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
	    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
	    {
	        MapPoint* pMP=vpMapPointMatches[i];
	        if(pMP)
	        {
	            if(!pMP->isBad())
	            {
	                pMP->ComputeDistinctiveDescriptors();
	                pMP->UpdateNormalAndDepth();
	            }
	        }
	    }

	    // Update connections in covisibility graph
	    mpCurrentKeyFrame->UpdateConnections();
	}



	void Recovery::UpdateLocalKeyFrames()
	{
	    // Each map point vote for the keyframes in which it has been observed
	    map<KeyFrame*,int> keyframeCounter;
	    for(int i=0; i<mCurrentFrame.N; i++)
	    {
	        if(mCurrentFrame.mvpMapPoints[i])
	        {
	            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
	            if(!pMP->isBad())
	            {
	                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
	                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
	                    keyframeCounter[it->first]++;
	            }
	            else
	            {
	                mCurrentFrame.mvpMapPoints[i]=NULL;
	            }
	        }
	    }

	    if(keyframeCounter.empty())
	        return;

	    int max=0;
	    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

	    mvRLocalKeyFrames.clear();
	    mvRLocalKeyFrames.reserve(3*keyframeCounter.size());

	    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
	    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
	    {
	        KeyFrame* pKF = it->first;

	        if(pKF->isBad())
	            continue;

	        if(it->second>max)
	        {
	            max=it->second;
	            pKFmax=pKF;
	        }

	        mvRLocalKeyFrames.push_back(it->first);
	        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
	    }


	    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
	    for(vector<KeyFrame*>::const_iterator itKF=mvRLocalKeyFrames.begin(), itEndKF=mvRLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
	    {
	        // Limit the number of keyframes
	        if(mvRLocalKeyFrames.size()>80)
	            break;

	        KeyFrame* pKF = *itKF;

	        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

	        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
	        {
	            KeyFrame* pNeighKF = *itNeighKF;
	            if(!pNeighKF->isBad())
	            {
	                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
	                {
	                    mvRLocalKeyFrames.push_back(pNeighKF);
	                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
	                    break;
	                }
	            }
	        }

	        const set<KeyFrame*> spChilds = pKF->GetChilds();
	        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
	        {
	            KeyFrame* pChildKF = *sit;
	            if(!pChildKF->isBad())
	            {
	                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
	                {
	                    mvRLocalKeyFrames.push_back(pChildKF);
	                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
	                    break;
	                }
	            }
	        }

	        KeyFrame* pParent = pKF->GetParent();
	        if(pParent)
	        {
	            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
	            {
	                mvRLocalKeyFrames.push_back(pParent);
	                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
	                break;
	            }
	        }

	    }
	    /*
	    if(pKFmax)
	    {
	        mpReferenceKF = pKFmax;
	        mCurrentFrame.mpReferenceKF = mpReferenceKF;
	    }*/
	}

	void Recovery::UpdateLocalPoints()
	{
	    mvRLocalMapPoints.clear();

	    for(vector<KeyFrame*>::const_iterator itKF = mvRLocalKeyFrames.begin(); itKF != mvRLocalKeyFrames.end(); itKF++)
	    {
	        KeyFrame* pKF = *itKF;
	        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

	        for(vector<MapPoint*>::const_iterator itMP = vpMPs.begin(); itMP != vpMPs.end(); itMP++)
	        {
	            MapPoint* pMP = *itMP;
	            if(!pMP)
	                continue;
	            if(pMP->mnTrackReferenceForFrameRecovery == mCurrentFrame.mnId)
	                continue;
	            if(!pMP->isBad())
	            {
	                mvRLocalMapPoints.push_back(pMP);
	                pMP->mnTrackReferenceForFrameRecovery = mCurrentFrame.mnId;
	            }
	        }
	    }
	}

	void Recovery::RecoveryMapByICP()
	{

	}
	void Recovery::RecoveryMapBySim3(g2o::Sim3 & mg2oScw)
	{

		mpMatchedKF->UpdateConnections();
		mpCurrentKeyFrame->UpdateConnections();
	    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
	    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
	    mvpCurrentConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
	    mvpCurrentConnectedKFs.push_back(mpMatchedKF);

	    LoopClosing::KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
	    CorrectedSim3[mpMatchedKF]=mg2oScw;
	    cv::Mat Twc = mpMatchedKF->GetPoseInverse();


   		{
        	// Get Map Mutex
	        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

	        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
	        {
	            KeyFrame* pKFi = *vit;

	            cv::Mat Tiw = pKFi->GetPose();

	            if(pKFi!=mpMatchedKF)
	            {
	                cv::Mat Tic = Tiw*Twc;
	                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
	                cv::Mat tic = Tic.rowRange(0,3).col(3);
	                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
	                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
	                //Pose corrected with the Sim3 of the loop closure
	                CorrectedSim3[pKFi]=g2oCorrectedSiw;
	            }

	            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
	            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
	            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
	            //Pose without correction
	            NonCorrectedSim3[pKFi]=g2oSiw;
	        }

	        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
	        for(LoopClosing::KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
	        {
	            KeyFrame* pKFi = mit->first;
	            g2o::Sim3 g2oCorrectedSiw = mit->second;
	            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

	            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

	            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
	            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
	            {
	                MapPoint* pMPi = vpMPsi[iMP];
	                if(!pMPi)
	                    continue;
	                if(pMPi->isBad())
	                    continue;
	                if(pMPi->mnCorrectedByKF==mpMatchedKF->mnId)
	                    continue;

	                // Project with non-corrected pose and project back with corrected pose
	                cv::Mat P3Dw = pMPi->GetWorldPos();
	                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
	                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

	                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
	                pMPi->SetWorldPos(cvCorrectedP3Dw);
	                pMPi->mnCorrectedByKF = mpMatchedKF->mnId;
	                pMPi->mnCorrectedReference = pKFi->mnId;
	                pMPi->UpdateNormalAndDepth();
	            }

	            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
	            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
	            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
	            double s = g2oCorrectedSiw.scale();

	            eigt *=(1./s); //[R t/s;0 1]

	            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

	            pKFi->SetPose(correctedTiw);

	            // Make sure connections are updated
	            pKFi->UpdateConnections();
	        }

	        // Start Loop Fusion
	        // Update matched map points and replace if duplicated
	        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
	        {
	            if(mvpCurrentMatchedPoints[i])
	            {
	                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
	                MapPoint* pCurMP = mpMatchedKF->GetMapPoint(i);
	                if(pCurMP)
	                    pCurMP->Replace(pLoopMP);
	                else
	                {
	                    mpMatchedKF->AddMapPoint(pLoopMP,i);
	                    pLoopMP->AddObservation(mpMatchedKF,i);
	                    pLoopMP->ComputeDistinctiveDescriptors();
	                }
	            }
	        }

    	}

	    // Project MapPoints observed in the neighborhood of the loop keyframe
	    // into the current keyframe and neighbors using corrected poses.
	    // Fuse duplications.
	    
	    //SearchAndFuse(CorrectedSim3);

        ORBmatcher matcherFuse(0.8);

        for(LoopClosing::KeyFrameAndPose::const_iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend;mit++)
        {
            KeyFrame* pKF = mit->first;
            //if(pKF->isBad())
            //    continue;
            g2o::Sim3 g2oScw = mit->second;
            cv::Mat cvScw = Converter::toCvMat(g2oScw);

            vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
            matcherFuse.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);

            // Get Map Mutex
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
            const int nLP = mvpLoopMapPoints.size();
            for(int i=0; i<nLP;i++)
            {
                MapPoint* pRep = vpReplacePoints[i];
                if(pRep)
                {
                    pRep->Replace(mvpLoopMapPoints[i]);
                }
            }
        }

	    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
	    map<KeyFrame*, set<KeyFrame*> > LoopConnections;

	    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
	    {
	        KeyFrame* pKFi = *vit;
	        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

	        // Update connections. Detect new links.
	        pKFi->UpdateConnections();
	        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
	        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
	        {
	            LoopConnections[pKFi].erase(*vit_prev);
	        }
	        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
	        {
	            LoopConnections[pKFi].erase(*vit2);
	        }
	    }

	    // Optimize graph
	    
	    Optimizer::OptimizeEssentialGraph(mpMap, mpCurrentKeyFrame, mpMatchedKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, false);

	    mpMap->InformNewBigChange();
	    
	    // Add loop edge
	    //mpMatchedKF->ChangeParent(mpCurrentKeyFrame);
	    mpMatchedKF->AddLoopEdge(mpCurrentKeyFrame);
	    mpCurrentKeyFrame->AddLoopEdge(mpMatchedKF);
	    /*
	    // Launch a new thread to perform Global Bundle Adjustment
	    mbRunningGBA = true;
	    mbFinishedGBA = false;
	    mbStopGBA = false;
	    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKeyFrame->mnId);*/
	}

	bool Recovery::DetectInOldMap(g2o::Sim3 & mg2oScw)
	{
		ORBmatcher matcher(0.75,true);
		for(vector<KeyFrame*>::iterator itKF = mvRLocalKeyFramesWhenBreak.begin(); itKF != mvRLocalKeyFramesWhenBreak.end(); itKF++)
	    {
	    	KeyFrame* curKF = *itKF;
	    	if(!curKF || curKF->isBad())
	    		continue;
    	    vector<MapPoint*> vpMapPointMatches;
    		int bias = mpCurrentKeyFrame->FrameSN - curKF->FrameSN;
    		if(abs(bias)>10)
    			continue;

    		//int inliers = 0;
    		int nmatches = matcher.SearchByBoW(curKF, mpCurrentKeyFrame,vpMapPointMatches);
    		
    		if(nmatches < 30)
    			continue;

    		cout<<mpCurrentKeyFrame->FrameSN<<" detect "<<nmatches<<" in the old map "<<curKF->FrameSN <<endl;
			//ComputeSim3(mpCurrentKeyFrame,curKF,vpMapPointMatches);
    			
   			bool mbFixScale =false;
		    vector<bool> vbInliers;
		    int nInliers;
		    bool bNoMore;
		    Sim3Solver* pSolver = new Sim3Solver(curKF, mpCurrentKeyFrame, vpMapPointMatches,mbFixScale);
		    pSolver->SetRansacParameters(0.99,20,300);
		    cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

		    if(Scm.empty() && nInliers < 20)
		    	continue;

	        //cout<<"!Scm.empty()"<<endl;
	        vector<MapPoint*> tempMapPoint(vpMapPointMatches.size(), static_cast<MapPoint*>(NULL));
            for(size_t j=0; j<vbInliers.size(); j++)
            {
                if(vbInliers[j])
                {
                  	tempMapPoint[j]=vpMapPointMatches[j];
                }
            }
            cv::Mat R = pSolver->GetEstimatedRotation();
	        cv::Mat t = pSolver->GetEstimatedTranslation();
	        const float s = pSolver->GetEstimatedScale();
	        matcher.SearchBySim3(curKF, mpCurrentKeyFrame, tempMapPoint,s,R,t,7.5);
	        g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
    		nInliers = Optimizer::OptimizeSim3(curKF, mpCurrentKeyFrame,tempMapPoint, gScm, 10, mbFixScale);
			
			if(nInliers < 20)
				continue;

            g2o::Sim3 gSmw(Converter::toMatrix3d(mpCurrentKeyFrame->GetRotation()),Converter::toVector3d(mpCurrentKeyFrame->GetTranslation()),1.0);	       
            //g2o::Sim3  tempSim3 = gScm*gSmw; //g2o::Sim3 
            //cout<<"g2o::Sim3 gSmw"<<endl;
            mg2oScw = gScm*gSmw;
            //cout<<"g2o::Sim3 mg2oScw"<<endl;
            cv::Mat mScw = Converter::toCvMat(mg2oScw);
            mvpCurrentMatchedPoints = tempMapPoint;

            vector<KeyFrame*> vpLoopConnectedKFs = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();
		    vpLoopConnectedKFs.push_back(mpCurrentKeyFrame);

		    mvpLoopMapPoints.clear();
		    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
		    {
		        KeyFrame* pKF = *vit;
		        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
		        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
		        {
		            MapPoint* pMP = vpMapPoints[i];
		            if(pMP)
		            {
		                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKeyFrame->mnId)
		                {
		                    mvpLoopMapPoints.push_back(pMP);
		                    pMP->mnLoopPointForKF=mpCurrentKeyFrame->mnId;
		                }
		            }
		        }
		    }

		   	matcher.SearchByProjection(curKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10); //mvpLoopMapPoints

		    // If enough matches accept Loop
		    int nTotalMatches = 0;
		    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
		    {
		        if(mvpCurrentMatchedPoints[i])
		            nTotalMatches++;
		    }
		    if(nTotalMatches >= 80)
		    {
		    	mpMatchedKF = curKF;
		    	cout<<mvpLoopMapPoints.size()<<" compute SIM3 succesfully!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!:   "<<nTotalMatches<<endl;
		        return true;
		    }
	    }
	    return false;	
	}



	/*
		vector<cv::Point2f> mvP2D;
        vector<cv::Point3f> mvP3D;
        
        int nmatches = matcher.SearchByGMS(mCurrentFrame,mpLastKeyFrame,mvP3D,mvP2D);//
        int inliers = mvP3D.size();
    */

    /*
        cv::Mat rvec, tvec, inliersPnP;
        int iterationsCount = 200;      // number of Ransac iterations.
        float reprojectionError = 5.99;  // maximum allowed distance to consider it an inlier.
        double confidence = 0.99;        // ransac successful confidence.
        cv::solvePnPRansac(mvP3D, mvP2D, mCurrentFrame.mK, mCurrentFrame.mDistCoef, rvec, tvec, false, iterationsCount, reprojectionError, confidence, inliersPnP,cv::SOLVEPNP_EPNP); //CV_EPNP

        cv::Mat R;
        cv::Rodrigues(rvec, R); //罗德里格斯变换
        
        cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
        R.copyTo(Tcw.rowRange(0,3).colRange(0,3));
        tvec.copyTo(Tcw.rowRange(0,3).col(3));
        
        mCurrentFrame.SetPose(Tcw); 
        //mCurrentFrame.SetPose(mpLastKeyFrame->GetPose());
    */
}//namespace Dual_SLAM