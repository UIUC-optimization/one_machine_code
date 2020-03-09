#include "OneMachineDP.h"

void OneMachDPCritPath::initialize() 
{
	if (mOneMachDPData != nullptr) {
		mJsonFile = mOneMachDPData->mJsonFile;
	}
	mPosInPathByJob.resize(mOneMachDPData->numJobs);
	mIndInPathByPos.resize(mOneMachDPData->numJobs);
	mIndexedJob.resize(mOneMachDPData->numJobs);
}

void OneMachDPCritPath::main(OneMachDPNode* node) 
{
	mCurNode = node;
	mCurFeaSol = mCurNode->mFeaSol;
	mLgstPathesByJob.resize(mOneMachDPData->numJobs);
	mMaxLgthToJob.resize(mOneMachDPData->numJobs, -1);
	mCanBeInCritPath.resize(mOneMachDPData->numJobs, 0);
	mAllCritPath.clear();
	fillPosAndIndex();
	auto iter = mCurNode->mSolPath.begin();
	auto end = mCurNode->mSolPath.end();
	while (iter != end) {
		findAllLgestPathToJob(*iter);
		iter++;
	}
	findAllCritPath();
	//filterCritPath();
	findValidCritPath();
	mLgstPathesByJob.clear();
	mMaxLgthToJob.clear();
	mCanBeInCritPath.clear();
}

void OneMachDPCritPath::fillPosAndIndex() 
{
	auto iter = mCurNode->mSolPath.begin();
	auto end = mCurNode->mSolPath.end();
	int count = 0;
	while (iter != end) {
		mPosInPathByJob[(*iter)->jobIndex] = count;
		mIndInPathByPos[count] = (*iter)->jobIndex;
		mIndexedJob[(*iter)->jobIndex] = *iter;
		iter++;
		count++;
	}
}

int OneMachDPCritPath::findAllLgestPathToJob(JobStep* curJob) 
{
	int maxPathLgth;
	int curPathLgth;
	int curIndex = curJob->jobIndex;
	bool hasLastStepAsPred = false;
	vector<int> maxLgthIndices;
	// If current jobstep has already calculated, just return
	if (mMaxLgthToJob[curIndex] != -1) {
		return mMaxLgthToJob[curIndex];
	}
	// If current jobstep is the fisrt, then only one path is possible
	if (mPosInPathByJob[curIndex] == 0) {
		mMaxLgthToJob[curIndex] = curJob->head;
		list<JobStep*> curPath;
		curPath.push_back(curJob);
		mLgstPathesByJob[curIndex].push_back(curPath);
		//mLgstPathesByJob[curIndex].push_back(list<JobStep*>());
		//mLgstPathesByJob[curIndex].back().push_back(curJob);
		// If current job step can be used as the last step in critical path, record that
		if (curJob->head + curJob->body + curJob->tail == mCurFeaSol) 
			mCanBeInCritPath[curIndex] = 1;

		return mMaxLgthToJob[curIndex];
	}
	// Otherwise, best can come from 1) Previous jobstep, 2) Anyone of the DPC predecessors or 3) current node itself
	int curPos = mPosInPathByJob[curIndex];
	int prevIndex = mIndInPathByPos[curPos - 1];

	// Set max to be from previous jobstep
	maxPathLgth = findAllLgestPathToJob(mIndexedJob[prevIndex]);
	maxPathLgth += (mIndexedJob[prevIndex]->body + mOneMachDPData->getDelay(prevIndex, curIndex));
	// Check if job prevIndex can be last in critical path
	//if (mCanBeInCritPath[prevIndex] == 0)
	maxLgthIndices.push_back(prevIndex);
	
	// If predecessor exists, check DPCs
	if (!mCurNode->mAllPreds[curIndex].empty()) {
		auto iter = mCurNode->mAllPreds[curIndex].begin();
		auto end = mCurNode->mAllPreds[curIndex].end();
		JobStep* predStep;
		while (iter != end) {
			// Only DPCs matter
			if ((*iter)->delay != 0) {
				predStep = (*iter)->from;
				curPathLgth = findAllLgestPathToJob(predStep);
				curPathLgth += (predStep->body + mOneMachDPData->getDelay(predStep->jobIndex, curIndex));
				if (curPathLgth > maxPathLgth) {
					maxLgthIndices.clear();
					maxPathLgth = curPathLgth;
				}
				int predStepIndex = predStep->jobIndex;
				if (predStepIndex == prevIndex) {
					iter++;
					continue;
				}
				//if (curPathLgth >= maxPathLgth && mCanBeInCritPath[predStepIndex] == 0)
				if (curPathLgth >= maxPathLgth) {
					maxLgthIndices.push_back(predStepIndex);
				}
			} else {
				break;
			}
			iter++;
		}
	}
	// Last, check current head
	if (maxPathLgth < curJob->head) {
		// Longest path is starting from current jobstep itself
		maxPathLgth = curJob->head;
		mMaxLgthToJob[curIndex] = maxPathLgth;

		if (curJob->head + curJob->body + curJob->tail == mCurFeaSol)
			mCanBeInCritPath[curIndex] = 1;

		mLgstPathesByJob[curIndex].push_back(list<JobStep*>());
		mLgstPathesByJob[curIndex].back().push_back(curJob);
		
		return mMaxLgthToJob[curIndex];
	}

	mMaxLgthToJob[curIndex] = maxPathLgth;
	if (maxLgthIndices.empty())
		//mCanBeInCritPath[curIndex] = -1;
		mCanBeInCritPath[curIndex] = 0;
	else {
		for (int maxLgthIndex : maxLgthIndices) {
			if (curIndex == maxLgthIndex) continue;
			auto pathIter = mLgstPathesByJob[maxLgthIndex].begin();
			auto pathEnd = mLgstPathesByJob[maxLgthIndex].end();
			for (; pathIter != pathEnd; pathIter++) {
				list<JobStep*> curPath = (*pathIter);
				curPath.push_back(curJob);
				mLgstPathesByJob[curIndex].push_back(curPath);
			}
		}
		if (maxPathLgth + curJob->body + curJob->tail == mCurFeaSol)
			mCanBeInCritPath[curIndex] = 1;
	}
	
	return mMaxLgthToJob[curIndex];
}

void OneMachDPCritPath::findAllCritPath() 
{
	int jobIndex;
	for (int i = 0; i < mOneMachDPData->numJobs; i++) {
		jobIndex = mIndInPathByPos[i];
		// If current job is the last job in critical path, then add all its longest pathes to critical path list
		if (mCanBeInCritPath[jobIndex] == 1) {
			auto iter = mLgstPathesByJob[jobIndex].begin();
			auto end = mLgstPathesByJob[jobIndex].end();
			while (iter != end) {
				mAllCritPath.push_back((*iter));
				//(*iter).clear();
				iter++;
			}
		}
		//mLgstPathesByJob[jobIndex].clear();
	}
}

/************************************************************************************************************
 * Allow each jobstep to be first in critical path once														*
 ************************************************************************************************************/
void OneMachDPCritPath::filterCritPath() 
{
	int curTopIndex, curPathLgth;
	auto pathIter = mAllCritPath.begin();
	vector<int> canBeTopCritPath;
	vector<list<JobStep*>> candidatPathes;
	canBeTopCritPath.resize(mOneMachDPData->numJobs, 0);
	candidatPathes.resize(mOneMachDPData->numJobs);
	for (auto path : mAllCritPath) {
		curTopIndex = path.front()->jobIndex;
		curPathLgth = path.size();
		// take the first path with curTopIndex as first job
		if (canBeTopCritPath[curTopIndex] == 0) {
			canBeTopCritPath[curTopIndex] = curPathLgth;
			candidatPathes[curTopIndex] = path;
		}
	}
	mAllCritPath.clear();
	for (int i = 0; i < mOneMachDPData->numJobs; i++) {
		if (canBeTopCritPath[i] > 0)
			mAllCritPath.push_back(candidatPathes[i]);
	}
}

/************************************************************************************************************
 * Remove critical pathes that can be extended to preceding jobs											*
 ************************************************************************************************************/
void OneMachDPCritPath::findValidCritPath() 
{
	int curPos, curIndex;
	JobStep* curStep;
	JobStep* preStep;
	auto pathIter = mAllCritPath.begin();
	auto pathEnd = mAllCritPath.end();
	while (pathIter != pathEnd) {
		curStep = (*pathIter).front();
		curIndex = curStep->jobIndex;
		curPos = mPosInPathByJob[curIndex];
		if (curPos == 0) {
			pathIter++;
			continue;
		}
		preStep = mIndexedJob[mIndInPathByPos[curPos - 1]];
		// If there is a gap before the first job of critical path
		if (mMaxLgthToJob[curIndex] - mMaxLgthToJob[preStep->jobIndex] - preStep->body > 0) {
			bool byDPC = false;
			auto iter = mCurNode->mAllPreds[curIndex].begin();
			auto end = mCurNode->mAllPreds[curIndex].end();
			while (iter != end) {
				if ((*iter)->delay == 0) {
					iter++;
					continue;
				}
				if (mMaxLgthToJob[curIndex] - mMaxLgthToJob[(*iter)->from->jobIndex] - (*iter)->delay == 0) {
					byDPC = true;
					break;
				}
				iter++;
			}
			if (byDPC) {
				mAllCritPath.erase(pathIter);
				continue;
			}
		} else {
			// if no gap, we simply ignore such critical path
			mAllCritPath.erase(pathIter);
			continue;
		}
		pathIter++;
	}
}

void OneMachDPCritPath::clearPathes() 
{
	mAllCritPath.clear();
}