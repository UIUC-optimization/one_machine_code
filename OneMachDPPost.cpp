#include "OneMachineDP.h"

void OneMachDPPost::initialize()
{
	if (mOneMachDPData != nullptr) {
		mJsonFile = mOneMachDPData->mJsonFile;
	}
	mNumHeadUpdts = mNumTailUpdts = 0;
}

/************************************************************************************************************
 * Postprocessing of feasible schedules																		*
 ************************************************************************************************************/
bool OneMachDPPost::main(OneMachDPNode* node)
{
	if (mOneMachDPData->numInitFix == 0)
		return false;
	bool headChanged = false, tailChanged = false;
	bool hasTailUpdates = false;
	list<list<JobStep*>>::iterator pathIter;
	list<list<JobStep*>>::iterator pathEnd;

	printf("Start post processing...\n");

	pathIter = mOneMachDPData->mCritPathes->mAllCritPath.begin();
	pathEnd = mOneMachDPData->mCritPathes->mAllCritPath.end();
	
	while (pathIter != pathEnd) {
		mCurPath = &*pathIter;
		mCurJobErased = true;
		while (mCurJobErased) {
			mCurJobErased = false;
			// Set to break if a post processing is actually applied
			tailChanged = chkPrecedPost(node);
			hasTailUpdates = (!hasTailUpdates) ? tailChanged : hasTailUpdates;		// If there has been tails updated, then hasTailUpdates will be true
		}
		pathIter++;
	}
	if (hasTailUpdates) {
		return hasTailUpdates;
	} else {
		pathIter = mOneMachDPData->mCritPathes->mAllCritPath.begin();
		while (pathIter != pathEnd) {
			mCurPath = &*pathIter;
			mCurJobErased = true;
			while (mCurJobErased) {
				mCurJobErased = false;
				headChanged = chkSuccedPost(node);
			}
			pathIter++;
		}
	}
	return hasTailUpdates;
}

/************************************************************************************************************
 * Proposition 3.3, tail updates																			*
 ************************************************************************************************************/
bool OneMachDPPost::chkPrecedPost(OneMachDPNode* node)
{
	auto iter = mCurPath->rbegin();
	auto end = mCurPath->rend();
	auto pre = iter;
	int lastTail = (*iter)->tail;
	int curIndex, preIndex;
	int newTail;
	int gap;
	JobStep* iterJob;
	preIndex = (*pre)->jobIndex;
	iter++;
	// Search from the last node of critical pass for essential precedence arc
	while (iter != end) {
		curIndex = (*iter)->jobIndex;
		iterJob = *iter;

		/* Only check EPC. */
		if (node->haveDPC(curIndex, preIndex)) {
			// Check all job step after curIndex, if head of job step i is smaller than sacheduled time of curIndex, AND
			// job step i is not a successor of curIndex
			for (auto i = mCurPath->rbegin(); i != pre; i++) {
				gap = node->numJobsBtwn(preIndex, (*i)->jobIndex);
				if ((*i)->head < node->mJobScheduled[preIndex] 
						&& !node->havePrecConstr(preIndex, (*i)->jobIndex, gap)) {
					return false;
				}
			}

			// Iterate all job steps after curIndex, and remove all from list
			auto jobToErase = mCurPath->end();
			jobToErase--;
			while ((*jobToErase)->jobIndex != preIndex) {
				node->addFix(iterJob, *jobToErase, 0);
				mCurPath->erase(jobToErase);
				// Erase moves the iterator forward, need to counter this with --
				jobToErase = mCurPath->end();
				jobToErase--;
			}
			mCurPath->erase(jobToErase);
			mCurJobErased = true;

			newTail = node->mFeaSol - node->mJobScheduled[curIndex] - iterJob->body;
			if (node->mUpdatedTail[curIndex] != newTail) {
				node->mUpdatedTail[curIndex] = newTail;
				mOneMachDPData->getJobStep(curIndex)->tail = newTail;
				mOneMachDPData->getJobStep(curIndex)->isTailUpdated = true;
				//printf("Job step %d has updated tail %d.\n", curIndex, newTail);
				mNumTailUpdts++;
				return true;
			} else {
				// mOneMachDPData->getJobStep(curIndex)->tail == newTail means the update is already done.
				// If update is already done, then no need to continue with mCurPath, just return false
				// so the algorithm can continue the search
				return false;
			}
		}
		if ((*iter)->tail < lastTail) break;
		
		pre = iter; preIndex = curIndex;
		iter++;
	}
	return false;
}

/************************************************************************************************************
 * Proposition 3.4, head updates																			*
 ************************************************************************************************************/
bool OneMachDPPost::chkSuccedPost(OneMachDPNode* node)
{
	auto iter = mCurPath->begin();
	auto end = mCurPath->end();
	auto pre = iter;
	int firstHead = (*iter)->head;
	int curIndex, preIndex;
	int head, newHead;
	int gap;
	bool foundSpecialStep = false;
	preIndex = (*pre)->jobIndex;
	iter++;
	// Search from the first node of critical pass for essential precedence arc
	while (iter != end) {
		curIndex = (*iter)->jobIndex;

		/* Only check DPC. */
		if (node->haveDPC(preIndex, curIndex)) {
			// Check all job step before curIndex, if tail of job step i satisfies condition below, AND
			// job step i is not a predecessor of curIndex
			for (auto i = mCurPath->begin(); i != pre; i++) {
				gap = node->numJobsBtwn((*i)->jobIndex, preIndex);
				if ((*i)->tail < (node->mFeaSol - node->mJobScheduled[preIndex] - (*pre)->body) 
						&& !node->havePrecConstr((*i)->jobIndex, preIndex, gap)) {
					return false;
				}
			}
			// Post processing of proposition 3.4
			// Iterate all job steps before preIndex, and remove all from list
			int sum = 0;
			auto jobToErase = mCurPath->begin();
			head = (*jobToErase)->head;
			while ((*jobToErase)->jobIndex != preIndex) {
				sum += (*jobToErase)->body;
				node->addFix(*jobToErase, *iter, 0);
				mCurPath->erase(jobToErase);
				jobToErase = mCurPath->begin();
			}

			newHead = head + sum + (*pre)->body + (mOneMachDPData->mInitFixDPDelay)[preIndex][curIndex];
			// remove of preIndex has to be after its last use
			mCurPath->erase(jobToErase);
			mCurJobErased = true;
			if (node->mUpdatedHead[curIndex] != newHead) {
				node->mUpdatedHead[curIndex] = newHead;
				mOneMachDPData->getJobStep(curIndex)->head = newHead;
				//printf("Job step %d has updated head %d.\n", curIndex, newHead);
				mNumHeadUpdts++;
				return true;
			} else {
				return false;
			}
		}
		if ((*iter)->head < firstHead) break;

		pre = iter; preIndex = curIndex;
		iter++;
	}
	return false;
}