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

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include<mutex>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

using namespace std;

namespace ORB_SLAM2
{

KeyFrameDatabase::KeyFrameDatabase (ORBVocabulary *voc):
    mpVoc(voc)
{
    mvInvertedFile.resize(voc->size());
}


void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

//    cout << "KF " << pKF->mnId << ", mBowVec size = " << pKF->mBowVec.size() << endl;
    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}

void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}


vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore)
{
    set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<KeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->mnLoopQuery!=pKF->mnId)
                {
                    pKFi->mnLoopWords=0;
                    if(!spConnectedKeyFrames.count(pKFi))
                    {
                        pKFi->mnLoopQuery=pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;

            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        KeyFrame* pBestKF = pKFi;
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pKF2->mLoopScore;
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;
}

    vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidateForKeyFrame(KeyFrame *kf) {
        {
            //1. Get the BoWVector for the current frame and iterate through all of its words
            //   The BoWVector is indexed by words and their associated weights
            //2. For each word, use the mvInvertedFile to find KF's that contain that specific word
            //3. For all those keyframes, put them in a vector as candidates, mark them as relocalization candidates for the current frame and count the number of shared words
            //4. Return if we have no keyframe relocalization candidates
            //5. Iterate through all keyframe relocalization candidates and find the maximum number of shared words between current frame and any relocalization keyframe
            //6. After that start computing their similarity scores ONLY for those that have greater similar words than 80%
            //  i. For each candidate, form an std::pair withe keyframe and BoVW SCORE while eliminating those kf's
            //     BoVW SCORE is calculated using the two BoWVec vectors which consist of visual words and their visibility scores
            //     So for each keyframe, we have reloc_query, reloc_words and reloc_score
            //7. Now iterate through the whole list of keyframes & their scores that share enough words
            //8. For each keyframe, query its covisibilty graph and get upto 10 keyframes neighbors for it
            //  i. Now using these neighbors, build an accumulative score for each keyframe by adding the scores of its neighbors
            //  ii. If a neighbor has more score than the keyframe, replace it and use the neighbor instead in the acc score keyframe pair
            //9. Using the 0.75 * bestScore, filter out the keyframes and return the rest as relocalization candidates







            list<KeyFrame*> lKFsSharingWords;
            // Search all keyframes that share a word with current frame
            {
                unique_lock<mutex> lock(mMutex);

                //1. Get the BoWVector for the current frame and iterate through all of its words
                //   The BoWVector is indexed by words and their associated weights

                for(DBoW2::BowVector::const_iterator vit=kf->mBowVec.begin(), vend=kf->mBowVec.end(); vit != vend; vit++)
                {
                    //2. For each word, use the mvInvertedFile to find KF's that contain that specific word
                    list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];
                    //get the list of all keyframes containing this word
                    for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
                    {
                        //3. For all those keyframes, put them in a vector as candidates, mark them as relocalization candidates for the current frame and count the number of shared words
                        KeyFrame* pKFi=*lit;
                        if(pKFi->mnRelocQuery!=kf->mnId)
                        {
                            pKFi->mnRelocWords=0;
                            pKFi->mnRelocQuery=kf->mnId;
                            lKFsSharingWords.push_back(pKFi);
                        }
                        pKFi->mnRelocWords++;
                    }
                }
            }
            if(lKFsSharingWords.empty())
            {
                //4. Return if we have no keyframe relocalization candidates
                cout << "No common words with any keyframe" << endl;
                return vector<KeyFrame*>();
            }
            // Only compare against those keyframes that share enough words
            int maxCommonWords=0;
            //5. Iterate through all keyframe relocalization candidates and find the maximum number of shared words between current frame and any relocalization keyframe
            for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
            {
                if((*lit)->mnRelocWords>maxCommonWords)
                    maxCommonWords=(*lit)->mnRelocWords;
            }
//            cout << "Maximum common words = " << maxCommonWords << endl;
            int minCommonWords = maxCommonWords*0.8f;
            list<pair<float,KeyFrame*> > lScoreAndMatch;
            int nscores=0;

            // Compute similarity score for BoVW
            for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi = *lit;
                //6. After that start computing their similarity scores ONLY for those that have greater similar words than 80%
                if(pKFi->mnRelocWords>minCommonWords)
                {
                    //  i. For each candidate, form an std::pair withe keyframe and BoVW SCORE while eliminating those kf's
                    //     BoVW SCORE is calculated using the two BoWVec vectors which consist of visual words and their visibility scores
                    //     So for each keyframe, we have reloc_query, reloc_words and reloc_score
                    nscores++;
                    float si = mpVoc->score(kf->mBowVec,pKFi->mBowVec);
                    pKFi->mRelocScore=si;
                    lScoreAndMatch.push_back(make_pair(si,pKFi));
                }
            }

            if(lScoreAndMatch.empty()) {
                cout << "Could not pass BoVW relocalization test" << endl;
                return vector<KeyFrame *>();
            }

            //9. Using the 0.75 * bestScore, filter out the keyframes and return the rest as relocalization candidates

            list<pair<float,KeyFrame*> > lAccScoreAndMatch;
            float bestAccScore = 0;

            // Lets now accumulate score by covisibility
            //7. Now iterate through the whole list of keyframes & their scores that share enough words
            for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
            {
                KeyFrame* pKFi = it->second;
                vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

                float bestScore = it->first;
                float accScore = bestScore;
                KeyFrame* pBestKF = pKFi;
                //8. For each keyframe, query its covisibilty graph and get upto 10 keyframes neighbors for it
                //  i. Now using these neighbors, build an accumulative score for each keyframe by adding the scores of its neighbors
                //  ii. If a neighbor has more score than the keyframe, replace it and use the neighbor instead in the acc score keyframe pair
                for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
                {
                    KeyFrame* pKF2 = *vit;
                    if(pKF2->mnRelocQuery!=kf->mnId)
                        continue;

                    accScore+=pKF2->mRelocScore;
                    if(pKF2->mRelocScore>bestScore)
                    {
                        pBestKF=pKF2;
                        bestScore = pKF2->mRelocScore;
                    }

                }
                lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
                if(accScore>bestAccScore)
                    bestAccScore=accScore;
            }

            // Return all those keyframes with a score higher than 0.75*bestScore
            float minScoreToRetain = 0.75f*bestAccScore;
            set<KeyFrame*> spAlreadyAddedKF;
            vector<KeyFrame*> vpRelocCandidates;
            vpRelocCandidates.reserve(lAccScoreAndMatch.size());
            for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
            {
                const float &si = it->first;
                if(si>minScoreToRetain)
                {
                    KeyFrame* pKFi = it->second;
                    if(!spAlreadyAddedKF.count(pKFi))
                    {
                        vpRelocCandidates.push_back(pKFi);
                        spAlreadyAddedKF.insert(pKFi);
                    }
                }
            }

            return vpRelocCandidates;
        }
    }


    vector<KeyFrame*> KeyFrameDatabase::DetectOverlappingKeyFrameCandidates(KeyFrame *kf)
    {
        const bool isDebugMode = false;
        {
            //1. Get the BoWVector for the current frame and iterate through all of its words
            //   The BoWVector is indexed by words and their associated weights
            //2. For each word, use the mvInvertedFile to find KF's that contain that specific word
            //3. For all those keyframes, put them in a vector as candidates, mark them as relocalization candidates for the current frame and count the number of shared words
            //4. Return if we have no keyframe relocalization candidates
            //5. Iterate through all keyframe relocalization candidates and find the maximum number of shared words between current frame and any relocalization keyframe
            //6. After that start computing their similarity scores ONLY for those that have greater similar words than 80%
            //  i. For each candidate, form an std::pair withe keyframe and BoVW SCORE while eliminating those kf's
            //     BoVW SCORE is calculated using the two BoWVec vectors which consist of visual words and their visibility scores
            //     So for each keyframe, we have reloc_query, reloc_words and reloc_score
            //7. Now iterate through the whole list of keyframes & their scores that share enough words
            //8. For each keyframe, query its covisibilty graph and get upto 10 keyframes neighbors for it
            //  i. Now using these neighbors, build an accumulative score for each keyframe by adding the scores of its neighbors
            //  ii. If a neighbor has more score than the keyframe, replace it and use the neighbor instead in the acc score keyframe pair
            //9. Using the 0.75 * bestScore, filter out the keyframes and return the rest as relocalization candidates







            list<KeyFrame *> lKFsSharingWords;
            // Search all keyframes that share a word with current frame
            {
                unique_lock<mutex> lock(mMutex);

                //1. Get the BoWVector for the current frame and iterate through all of its words
                //   The BoWVector is indexed by words and their associated weights

                for (DBoW2::BowVector::const_iterator vit = kf->mBowVec.begin(), vend = kf->mBowVec.end();
                     vit != vend; vit++) {
                    //2. For each word, use the mvInvertedFile to find KF's that contain that specific word
                    list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];
                    //get the list of all keyframes containing this word
                    for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
                        //3. For all those keyframes, put them in a vector as candidates, mark them as relocalization candidates for the current frame and count the number of shared words
                        KeyFrame *pKFi = *lit;
                        if (pKFi->mnRelocQuery != kf->mnId) {
                            pKFi->mnRelocWords = 0;
                            pKFi->mnRelocQuery = kf->mnId;
                            lKFsSharingWords.push_back(pKFi);
                        }
                        pKFi->mnRelocWords++;
                    }
                }
            }
            if (lKFsSharingWords.empty()) {
                //4. Return if we have no keyframe relocalization candidates
                if (isDebugMode)
                    cerr << "No common words with any keyframe" << endl;
                return vector<KeyFrame *>();
            }
            // Only compare against those keyframes that share enough words
            int maxCommonWords = 0;
            //5. Iterate through all keyframe relocalization candidates and find the maximum number of shared words between current frame and any relocalization keyframe
            for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end();
                 lit != lend; lit++) {
                if ((*lit)->mnRelocWords > maxCommonWords)
                    maxCommonWords = (*lit)->mnRelocWords;

                if (isDebugMode)
                    cout << "Common words for KF[" << (*lit)->mnId << "] = " << (*lit)->mnRelocWords << endl;
            }

            if (isDebugMode)
                cout << "Maximum common words = " << maxCommonWords << endl;
            int minCommonWords = maxCommonWords * 0.8f;
            list<pair<float, KeyFrame *> > lScoreAndMatch;
            int nscores = 0;

            std::vector<KeyFrame *> listOfKeyframes;
            // Compute similarity score for BoVW
            for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end();
                 lit != lend; lit++) {
                KeyFrame *pKFi = *lit;
                //6. After that start computing their similarity scores ONLY for those that have greater similar words than 80%
                if (pKFi->mnRelocWords > minCommonWords) {
                    //  i. For each candidate, form an std::pair withe keyframe and BoVW SCORE while eliminating those kf's
                    //     BoVW SCORE is calculated using the two BoWVec vectors which consist of visual words and their visibility scores
                    //     So for each keyframe, we have reloc_query, reloc_words and reloc_score
                    nscores++;
                    float si = mpVoc->score(kf->mBowVec, pKFi->mBowVec);
                    pKFi->mRelocScore = si;
                    lScoreAndMatch.push_back(make_pair(si, pKFi));

                    listOfKeyframes.push_back(pKFi);
                    if (isDebugMode)
                        cout << "KF [" << (*lit)->mnId << "] has similarity score = " << pKFi->mRelocScore << endl;

                } else
                if (isDebugMode)
                    cout << "Dropping KF [" << pKFi->mnId << "] for less than " << minCommonWords << " words" << endl;

            }

            if (lScoreAndMatch.empty()) {
                if (isDebugMode)

                    cout << "Could not pass BoVW relocalization test" << endl;
                return vector<KeyFrame *>();
            } else {
                if (isDebugMode)

                    cout << "Returning " << listOfKeyframes.size() << " keyframes" << endl;
                return listOfKeyframes;
            }



        }
        /*
            //9. Using the 0.75 * bestScore, filter out the keyframes and return the rest as relocalization candidates

            list<pair<float,KeyFrame*> > lAccScoreAndMatch;
            float bestAccScore = 0;

            // Lets now accumulate score by covisibility
            //7. Now iterate through the whole list of keyframes & their scores that share enough words
            for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
            {
                KeyFrame* pKFi = it->second;
                vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

                float bestScore = it->first;
                float accScore = bestScore;
                KeyFrame* pBestKF = pKFi;
                //8. For each keyframe, query its covisibilty graph and get upto 10 keyframes neighbors for it
                //  i. Now using these neighbors, build an accumulative score for each keyframe by adding the scores of its neighbors
                //  ii. If a neighbor has more score than the keyframe, replace it and use the neighbor instead in the acc score keyframe pair
                for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
                {
                    KeyFrame* pKF2 = *vit;
                    if(pKF2->mnRelocQuery!=kf->mnId)
                        continue;

                    accScore+=pKF2->mRelocScore;
                    if(pKF2->mRelocScore>bestScore)
                    {
                        pBestKF=pKF2;
                        bestScore = pKF2->mRelocScore;
                    }

                }
                lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
                if(accScore>bestAccScore)
                    bestAccScore=accScore;
            }

            // Return all those keyframes with a score higher than 0.75*bestScore
            float minScoreToRetain = 0.75f*bestAccScore;
            set<KeyFrame*> spAlreadyAddedKF;
            vector<KeyFrame*> vpRelocCandidates;
            vpRelocCandidates.reserve(lAccScoreAndMatch.size());
            for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
            {
                const float &si = it->first;
                if(si>minScoreToRetain)
                {
                    KeyFrame* pKFi = it->second;
                    if(!spAlreadyAddedKF.count(pKFi))
                    {
                        vpRelocCandidates.push_back(pKFi);
                        spAlreadyAddedKF.insert(pKFi);
                    }
                }
            }

            return vpRelocCandidates;
        }
         */
    }

vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)
{
    //1. Get the BoWVector for the current frame and iterate through all of its words
    //   The BoWVector is indexed by words and their associated weights
    //2. For each word, use the mvInvertedFile to find KF's that contain that specific word
    //3. For all those keyframes, put them in a vector as candidates, mark them as relocalization candidates for the current frame and count the number of shared words
    //4. Return if we have no keyframe relocalization candidates
    //5. Iterate through all keyframe relocalization candidates and find the maximum number of shared words between current frame and any relocalization keyframe
    //6. After that start computing their similarity scores ONLY for those that have greater similar words than 80%
    //  i. For each candidate, form an std::pair withe keyframe and BoVW SCORE while eliminating those kf's
    //     BoVW SCORE is calculated using the two BoWVec vectors which consist of visual words and their visibility scores
    //     So for each keyframe, we have reloc_query, reloc_words and reloc_score
    //7. Now iterate through the whole list of keyframes & their scores that share enough words
    //8. For each keyframe, query its covisibilty graph and get upto 10 keyframes neighbors for it
    //  i. Now using these neighbors, build an accumulative score for each keyframe by adding the scores of its neighbors
    //  ii. If a neighbor has more score than the keyframe, replace it and use the neighbor instead in the acc score keyframe pair
    //9. Using the 0.75 * bestScore, filter out the keyframes and return the rest as relocalization candidates





    list<KeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->mnRelocQuery!=F->mnId)
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
    {
//        cout << "No common words with any keyframe" << endl;
        return vector<KeyFrame*>();

    }

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }

//    cout << "Maximum common words = " << maxCommonWords << endl;

    int minCommonWords = maxCommonWords*0.8f;

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(F->mBowVec,pKFi->mBowVec);
            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFrame* pBestKF = pKFi;
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnRelocQuery!=F->mnId)
                continue;

            accScore+=pKF2->mRelocScore;
            if(pKF2->mRelocScore>bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;
    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        if(si>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

template<class Archive>
void KeyFrameDatabase::serialize(Archive &ar, const unsigned int version)
{
    // don't save associated vocabulary, KFDB restore by created explicitly from a new ORBvocabulary instance
    // inverted file

    //Optimization!
    ar & mvInvertedFile;
    // don't save mutex
}
template void KeyFrameDatabase::serialize(boost::archive::binary_iarchive&, const unsigned int);
template void KeyFrameDatabase::serialize(boost::archive::binary_oarchive&, const unsigned int);
    template void KeyFrameDatabase::serialize(boost::archive::text_oarchive&, const unsigned int);
    template void KeyFrameDatabase::serialize(boost::archive::text_iarchive&, const unsigned int);
} //namespace ORB_SLAM
