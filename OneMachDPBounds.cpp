#include "OneMachineDP.h"

void OneMachDPBounds::initialize() 
{
	if (mOneMachDPData != nullptr) {
		mNumJobSteps = mOneMachDPData->numJobs;
		mJsonFile = mOneMachDPData->mJsonFile;
	}
}

/************************************************************************************************************
 * Preemptive Scheduling (PREEMPTIVE longest tail heuristic algorithm, DPCs treated as common PC)			*
 ************************************************************************************************************/
int OneMachDPBounds::getLBStd(OneMachDPNode* node)
{
	int curTime = 0, stopTime, bound = 0;
	int count = 0, temp = 0;
	int jobSetAvalCount = 0;
	int schdJobCount = 0;
	int jobIndex;
	JobStep* step;
	JobStep* next;
	list<JobStep*> unschd;
	vector<int> jobScheduled;
	// availableJobSteps stores job steps that don't have unscheduled predecessor, ordered in ascending by head value
	minHeadQueue availableJobSteps(stepComp(3));
	// releaseJobSteps stores job steps that is not only available, but is released, ordered in descending by tail value
	maxTailQueue releaseJobSteps(stepComp(2));
	list<JobStep>::iterator iter = mOneMachDPData->mJobsData.begin();
	list<JobStep>::iterator end = mOneMachDPData->mJobsData.end();
	// move jobs to available pool if it does not have any predecessors
	while (iter != end) {
		step = &*iter;
		jobIndex = step->jobIndex;
		step->remTime = step->body;
		step->curHead = step->head;
		if (node->mAllPreds[jobIndex].empty()) {
			availableJobSteps.push(step);
			jobSetAvalCount++;
		}
		iter++;
	}
	jobScheduled.resize(mNumJobSteps, -1);
	// Clear solution path
	node->mLBSolPath.clear();
	// Sanity check. The first job should always be able to be added to queue
	step = availableJobSteps.top();
	if (curTime < step->curHead) curTime = step->curHead;
	// Main loop for JPS
	while (schdJobCount < mNumJobSteps) {
		/************************************************************************************************************
		* 1. Among all available jobs, find the ones that are released. Push into released job step poll.			*
		*	  If none, move curTime to the starting time of the first job in available list and try again.          *
		************************************************************************************************************/
		while (!availableJobSteps.empty()) {
			step = availableJobSteps.top();
			if (step->curHead <= curTime) {
				releaseJobSteps.push(step);
				availableJobSteps.pop();
			} else if (releaseJobSteps.empty()) {
				curTime = step->curHead;
				releaseJobSteps.push(step);
				availableJobSteps.pop();
			} else
				break;
		}
		/************************************************************************************************************
		* 2. - If there are more job steps that is in the available jobsteps queue, then there is a possibility	*
		*		that current job will be interrupted. stopTime is where that could happen (as it is the earliest	*
		*		starting time for the next job not in released job step queue).										*
		*	  - On the other hand, if available job step pool is empty, it means all available job steps are 		*
		*		released. In that case, set stop time to infinite if there is no unavailable jobs any more.			*
		*		Note that available empty doesn't mean all jobs released. Some jobs are still unavailable because	*
		*		of precedence constraints.																			*
		************************************************************************************************************/
		if (!availableJobSteps.empty()) {
			next = availableJobSteps.top();
			// Set stopTime to the curHead of next job step in available pool, preemption potential
			stopTime = next->curHead;
		} else {
			stopTime = ENDOFTIME;
		}
		/************************************************************************************************************
		* 3. Get the next job step to be scheduled																	*
		*		- If the job can be scheduled completely (curTime + step->remTime <= stopTime), Put the job in      *
		*		  existing schedule, keep track of scheduled jobs, and clean up.									*
		*      - If not, update remTime of the job, and move to the stopTime.										*
		************************************************************************************************************/
		step = releaseJobSteps.top();						// The step to schedule next
		node->mLBSolPath.push_back(step);
		if (curTime + step->remTime <= stopTime) {
			// The next jobstep cannot be added into release jobsteps queue after this;
			// On the other hand, current job is finished, update curTime to the finish time
			curTime += step->remTime;
			// Update bound if necessary
			if (curTime + step->tail > bound)
				bound = curTime + step->tail;
			jobScheduled[step->jobIndex] = curTime;		// Record the finish time of job step
			schdJobCount++;
			releaseJobSteps.pop();
			/********************************************************************************************************
			* 4. Update the release date of successors of the job being scheduled. Only when job step scheduled	*
			*    completely.																						*
			********************************************************************************************************/
			jobIndex = step->jobIndex;
			for (auto succJob = node->mAllSuccs[jobIndex].begin();
				succJob != node->mAllSuccs[jobIndex].end(); succJob++) {
				temp = curTime;
				if ((*succJob)->to->curHead < temp)
					(*succJob)->to->curHead = temp;
				unschd.push_back((*succJob)->to);
			}
		} else {
			// Current jobstep is not finished when the next available jobstep is released
			count++;
			step->remTime -= (stopTime - curTime);
			curTime = stopTime;
		}
		/************************************************************************************************************
		* 5. For all the unscheduled jobs, the ones whoes predecessors have all been scheduled is put				*
		*	  in a priority queue so they are sorted by head.														*
		************************************************************************************************************/
		if (!unschd.empty()) {
			auto unsJobIter = unschd.begin();
			while (unsJobIter != unschd.end()) {
				step = *unsJobIter;
				jobIndex = step->jobIndex;
				bool predDone = true;
				// Check the predecessor list for each unscheduled jobs to see if all predecessor scheduled
				for (auto predJob = node->mAllPreds[jobIndex].begin();
					predJob != node->mAllPreds[jobIndex].end(); predJob++) {
					int ind = (*predJob)->from->jobIndex;
					// if (node->mJobScheduled[ind] == 0) {
					if (jobScheduled[ind] == -1) {
						predDone = false;
						break;
					}
				}
				if (predDone) {
					availableJobSteps.push(step);
					jobSetAvalCount++;
				}
				unsJobIter++;
			}
			unschd.clear();
		}
	}
	// Reset curHead
	node->resetcurHead();
	node->mRexSol = bound;
	return bound;
}

int OneMachDPBounds::getLBStd(OneMachDPNode* node, list<JobStep*> &jobsToSchd)
{
	int curTime = 0, stopTime, bound = 0;
	int count = 0, temp = 0;
	int jobSetAvalCount = 0;
	int schdJobCount = 0;
	int jobIndex;
	JobStep* step;
	JobStep* next;
	list<JobStep*> unschd;
	vector<int> jobScheduled;
	// availableJobSteps stores job steps that don't have unscheduled predecessor, ordered in ascending by head value
	minHeadQueue availableJobSteps(stepComp(3));
	// releaseJobSteps stores job steps that is not only available, but is released, ordered in descending by tail value
	maxTailQueue releaseJobSteps(stepComp(2));
	list<JobStep*>::iterator iter = jobsToSchd.begin();
	list<JobStep*>::iterator end = jobsToSchd.end();
	// move jobs to available pool if it does not have any predecessors
	while (iter != end) {
		step = *iter;
		jobIndex = step->jobIndex;
		step->remTime = step->body;
		step->curHead = step->head;
		if (node->mAllPreds[jobIndex].empty()) {
			availableJobSteps.push(step);
			jobSetAvalCount++;
		}
		iter++;
	}
	jobScheduled.resize(mNumJobSteps, -1);

	// Sanity check. The first job should always be able to be added to queue
	step = availableJobSteps.top();
	if (curTime < step->curHead) curTime = step->curHead;
	// Main loop for JPS
	while (schdJobCount < jobsToSchd.size()) {
		/************************************************************************************************************
		* 1. Among all available jobs, find the ones that are released. Push into released job step poll.			*
		*	  If none, move curTime to the starting time of the first job in available list and try again.          *
		************************************************************************************************************/
		while (!availableJobSteps.empty()) {
			step = availableJobSteps.top();
			if (step->curHead <= curTime) {
				releaseJobSteps.push(step);
				availableJobSteps.pop();
			}
			else if (releaseJobSteps.empty()) {
				curTime = step->curHead;
				releaseJobSteps.push(step);
				availableJobSteps.pop();
			}
			else
				break;
		}
		/************************************************************************************************************
		* 2. - If there are more job steps that is in the available jobsteps queue, then there is a possibility	*
		*		that current job will be interrupted. stopTime is where that could happen (as it is the earliest	*
		*		starting time for the next job not in released job step queue).										*
		*	  - On the other hand, if available job step pool is empty, it means all available job steps are 		*
		*		released. In that case, set stop time to infinite if there is no unavailable jobs any more.			*
		*		Note that available empty doesn't mean all jobs released. Some jobs are still unavailable because	*
		*		of precedence constraints.																			*
		************************************************************************************************************/
		if (!availableJobSteps.empty()) {
			next = availableJobSteps.top();
			// Set stopTime to the curHead of next job step in available pool, preemption potential
			stopTime = next->curHead;
		}
		else {
			stopTime = ENDOFTIME;
		}
		/************************************************************************************************************
		* 3. Get the next job step to be scheduled																	*
		*		- If the job can be scheduled completely (curTime + step->remTime <= stopTime), Put the job in      *
		*		  existing schedule, keep track of scheduled jobs, and clean up.									*
		*      - If not, update remTime of the job, and move to the stopTime.										*
		************************************************************************************************************/
		step = releaseJobSteps.top();						// The step to schedule next

		if (curTime + step->remTime <= stopTime) {
			// The next jobstep cannot be added into release jobsteps queue after this;
			// On the other hand, current job is finished, update curTime to the finish time
			curTime += step->remTime;
			// Update bound if necessary
			if (curTime + step->tail > bound)
				bound = curTime + step->tail;
			jobScheduled[step->jobIndex] = curTime;		// Record the finish time of job step
			schdJobCount++;
			releaseJobSteps.pop();
			/********************************************************************************************************
			* 4. Update the release date of successors of the job being scheduled. Only when job step scheduled	*
			*    completely.																						*
			********************************************************************************************************/
			jobIndex = step->jobIndex;
			for (auto succJob = node->mAllSuccs[jobIndex].begin();
				succJob != node->mAllSuccs[jobIndex].end(); succJob++) {
				temp = curTime;
				if ((*succJob)->to->curHead < temp)
					(*succJob)->to->curHead = temp;
				unschd.push_back((*succJob)->to);
			}
		}
		else {
			// Current jobstep is not finished when the next available jobstep is released
			count++;
			step->remTime -= (stopTime - curTime);
			curTime = stopTime;
		}
		/************************************************************************************************************
		* 5. For all the unscheduled jobs, the ones whoes predecessors have all been scheduled is put				*
		*	  in a priority queue so they are sorted by head.														*
		************************************************************************************************************/
		if (!unschd.empty()) {
			auto unsJobIter = unschd.begin();
			while (unsJobIter != unschd.end()) {
				step = *unsJobIter;
				jobIndex = step->jobIndex;
				bool predDone = true;
				// Check the predecessor list for each unscheduled jobs to see if all predecessor scheduled
				for (auto predJob = node->mAllPreds[jobIndex].begin();
					predJob != node->mAllPreds[jobIndex].end(); predJob++) {
					int ind = (*predJob)->from->jobIndex;
					// if (node->mJobScheduled[ind] == 0) {
					if (jobScheduled[ind] == -1) {
						predDone = false;
						break;
					}
				}
				if (predDone) {
					availableJobSteps.push(step);
					jobSetAvalCount++;
				}
				unsJobIter++;
			}
			unschd.clear();
		}
	}
	// Reset curHead
	node->resetcurHead();
	return bound;
}

/************************************************************************************************************
 * Derive LB from critical path																				*
 ************************************************************************************************************/
int OneMachDPBounds::getLBFromSol(OneMachDPNode* node)
{
	int minHead, minTail, sumBody;
	int minHeadCrt, minTailCrt, sumBodyCrt;
	int maxLB = node->mRexSol;
	if (node->mCritStep != nullptr) {
		minHeadCrt = ENDOFTIME;
		minTailCrt = ENDOFTIME;
		sumBodyCrt = 0;
		for (auto iter = node->mCritPath.rbegin(); (*iter)->jobIndex != node->mCritStep->jobIndex; iter++) {
			minHeadCrt = ((*iter)->head < minHeadCrt) ? (*iter)->head : minHeadCrt;
			minTailCrt = ((*iter)->tail < minTailCrt) ? (*iter)->tail : minTailCrt;
			sumBodyCrt += (*iter)->body;
		}
	} else {
		minHeadCrt = ENDOFTIME;
		minTailCrt = ENDOFTIME;
		sumBodyCrt = 0;
		for (auto iter = node->mCritPath.rbegin(); iter != node->mCritPath.rend(); iter++) {
			minHeadCrt = ((*iter)->head < minHeadCrt) ? (*iter)->head : minHeadCrt;
			minTailCrt = ((*iter)->tail < minTailCrt) ? (*iter)->tail : minTailCrt;
			sumBodyCrt += (*iter)->body;
		}
	}
	return minHeadCrt + minTailCrt + sumBodyCrt;
}

/************************************************************************************************************
 * The longest tail heuristic algorithm for DPCs															*
 ************************************************************************************************************/
int OneMachDPBounds::getUB(OneMachDPNode* node)
{
	int curTime = 0;
	int maxTime = 0, temp = 0;
	int jobSetAvalCount = 0;
	int schdJobCount = 0;
	int jobIndex;
	JobStep* step;
	JobStep* prev;
	JobStep* tempStep;
	list<JobStep*> unschd;
	// availableJobSteps stores job steps that don't have unscheduled predecessor, ordered in ascending by head value
	minHeadQueue availableJobSteps(stepComp(3));
	// releaseJobSteps stores job steps that is not only available, but is released, ordered in descending by tail value
	maxTailQueue releaseJobSteps(stepComp(2));
	list<JobStep>::iterator iter = mOneMachDPData->mJobsData.begin();
	list<JobStep>::iterator end = mOneMachDPData->mJobsData.end();

	bool done = false;
	// Initialize: 1. move jobs to available pool if it does not have any predecessors
	//			   2. set job step curHead to head
	while (iter != end) {
		step = &*iter;
		jobIndex = step->jobIndex;
		step->curHead = step->head;
		if (node->mAllPreds[jobIndex].empty()) {
			availableJobSteps.push(step);
			jobSetAvalCount++;
		}
		iter++;
	}
	node->mJobScheduled.clear();
	node->mJobScheduled.resize(mNumJobSteps, -1);
	node->mSolPath.clear();
	// Sanity check. The first job should always be able to be added to queue
	step = availableJobSteps.top();
	if (curTime < step->curHead) curTime = step->curHead;

	while (schdJobCount < mNumJobSteps)
	{
		/************************************************************************************************************
		 * 1. Among all available jobs, find the ones that are released. Push into released job step poll.			*
		 *	  If none, move curTime to the starting time of the first job in available list and try again.          *
		 ************************************************************************************************************/
		while (!availableJobSteps.empty()) {
			step = availableJobSteps.top();
			if (step->curHead <= curTime) {
				releaseJobSteps.push(step);
				availableJobSteps.pop();
			} else if (releaseJobSteps.empty()) {
				curTime = step->curHead;
				releaseJobSteps.push(step);
				availableJobSteps.pop();
			} else
				break;
		}
		/************************************************************************************************************
		 * 2. Get the next job to schedule. Note that curTime should not be smaller than the head of step because   *
		 *	  previous section should have already updated curTime properly.                                        *
		 ************************************************************************************************************/
		step = releaseJobSteps.top();						// The step to schedule next		
		node->mJobScheduled[step->jobIndex] = curTime;		// Record starting time of job step
		curTime += step->body;
		if (curTime + step->tail > maxTime) {
			maxTime = curTime + step->tail;
		}
		
		/************************************************************************************************************
		 * 3. Update the release date of successors of the job being scheduled, if necessary.                       *
		 ************************************************************************************************************/
		jobIndex = step->jobIndex;
		for (auto succJob = node->mAllSuccs[jobIndex].begin(); 
				succJob != node->mAllSuccs[jobIndex].end(); succJob++) {
			temp = (*succJob)->delay + curTime;
			if ((*succJob)->to->curHead < temp)
				(*succJob)->to->curHead = temp;
			unschd.push_back((*succJob)->to);
		}
		/************************************************************************************************************
		 * 4. Put the job in existing schedule, keep track of scheduled jobs, and clean up.                         *
		 ************************************************************************************************************/
		node->mSolPath.push_back(step);
		schdJobCount++;
		releaseJobSteps.pop();
		prev = step;

		/************************************************************************************************************
		* 5. For all the unscheduled jobs, the ones whoes predecessors have all been scheduled is put				*
		*	  in a priority queue so they are sorted by head.														*
		************************************************************************************************************/
		if (!unschd.empty()) {
			auto unsJobIter = unschd.begin();
			while (unsJobIter != unschd.end()) {
				step = *unsJobIter;
				jobIndex = step->jobIndex;
				bool predDone = true;
				// Check the predecessor list for each unscheduled jobs to see if all predecessor scheduled
				for (auto predJob = node->mAllPreds[jobIndex].begin();
					predJob != node->mAllPreds[jobIndex].end(); predJob++) {
					int ind = (*predJob)->from->jobIndex;
					if (node->mJobScheduled[ind] == -1) {
						predDone = false;
						break;
					}
				}
				if (predDone) {
					availableJobSteps.push(step);
					jobSetAvalCount++;
				}
				unsJobIter++;
			}
			unschd.clear();
		}
	}
	if (jobSetAvalCount != mNumJobSteps)
		throw ERROR << "Not all job step can be scheduled, check precedence constraints.";
	int curSol = mOneMachDPData->calSolution(node);
	// Reset curHead
	node->resetcurHead();
	node->mFeaSol = curSol;
	return curSol;
}

/************************************************************************************************************
 * OneMachDPBounds::getUBMod(OneMachDPNode* node):															*
 * The longest tail heuristic algorithm for DPCs, with modified procedure									*
 ************************************************************************************************************/
// Type 1
int OneMachDPBounds::getUBMod(OneMachDPNode* node)
{
	int curTime = 0;
	int maxTime = 0, temp = 0;
	int jobSetAvalCount = 0;
	int schdJobCount = 0;
	int jobIndex, updtIndex;
	JobStep* step;
	JobStep* prev;
	JobStep* tempStep;
	list<JobStep*> unschd;
	// availableJobSteps stores job steps that don't have unscheduled predecessor, ordered in ascending by head value
	minHeadQueue availableJobSteps(stepComp(3));
	// releaseJobSteps stores job steps that is not only available, but is released, ordered in descending by tail value
	maxTailQueue releaseJobSteps(stepComp(2));
	list<JobStep>::iterator iter = mOneMachDPData->mJobsData.begin();
	list<JobStep>::iterator end = mOneMachDPData->mJobsData.end();

	bool done = false;
	// Initialize: 1. move jobs to available pool if it does not have any predecessors
	//			   2. set job step curHead to head
	while (iter != end) {
		step = &*iter;
		jobIndex = step->jobIndex;
		step->curHead = step->head;
		if (node->mAllPreds[jobIndex].empty()) {
			availableJobSteps.push(step);
			jobSetAvalCount++;
		}
		iter++;
	}
	//int *jobsScheduled = (int*)calloc(mNumJobSteps,sizeof(int));
	node->mJobScheduled.clear();
	node->mJobScheduled.resize(mNumJobSteps, -1);
	node->mSolPath.clear();
	// Sanity check. The first job should always be able to be added to queue
	step = availableJobSteps.top();
	if (curTime < step->curHead) curTime = step->curHead;

	while (schdJobCount < mNumJobSteps)
	{
		/************************************************************************************************************
		 * 1. Among all available jobs, find the ones that are released. Push into released job step poll.			*
		 *	  If none, move curTime to the starting time of the first job in available list and try again.          *
		 ************************************************************************************************************/
		while (!availableJobSteps.empty()) {
			step = availableJobSteps.top();
			if (step->curHead <= curTime) {
				releaseJobSteps.push(step);
				availableJobSteps.pop();
			} else if (releaseJobSteps.empty()) {
				curTime = step->curHead;
				releaseJobSteps.push(step);
				availableJobSteps.pop();
			} else
				break;
		}
		/************************************************************************************************************
		 * 2. Get the next job to schedule. Note that curTime should not be smaller than the head of step because   *
		 *	  previous section should have already updated curTime properly.                                        *
		 ************************************************************************************************************/
		step = releaseJobSteps.top();						// The step to schedule next
		node->mJobScheduled[step->jobIndex] = curTime;		// Record starting time of job step
		curTime += step->body;
		if (curTime + step->tail > maxTime) {
			maxTime = curTime + step->tail;
		}

		/************************************************************************************************************
		 * 3. Update the release date of successors of the job being scheduled, if necessary.                       *
		 ************************************************************************************************************/
		jobIndex = step->jobIndex;
		for (auto succJob = node->mAllSuccs[jobIndex].begin();
			succJob != node->mAllSuccs[jobIndex].end(); succJob++) {
			temp = (*succJob)->delay + curTime;
			if ((*succJob)->to->curHead < temp) {
				(*succJob)->to->curHead = temp;
				updtIndex = (*succJob)->to->jobIndex;
			}
			unschd.push_back((*succJob)->to);
		}
		/************************************************************************************************************
		 * 4. Put the job in existing schedule, keep track of scheduled jobs, and clean up.                         *
		 ************************************************************************************************************/
		node->mSolPath.push_back(step);
		schdJobCount++;
		releaseJobSteps.pop();
		prev = step;

		/************************************************************************************************************
		 * 5. For all the unscheduled jobs, the ones whoes predecessors have all been scheduled is put				*
		 *	  in a priority queue so they are sorted by head.														*
		 ************************************************************************************************************/
		if (!unschd.empty()) {
			auto unsJobIter = unschd.begin();
			while (unsJobIter != unschd.end()) {
				step = *unsJobIter;
				jobIndex = step->jobIndex;
				bool predDone = true;
				// Check the predecessor list for each unscheduled jobs to see if all predecessor scheduled
				for (auto predJob = node->mAllPreds[jobIndex].begin();
					predJob != node->mAllPreds[jobIndex].end(); predJob++) {
					int ind = (*predJob)->from->jobIndex;
					if (node->mJobScheduled[ind] == -1) {
						predDone = false;
						break;
					}
				}
				if (predDone) {
					availableJobSteps.push(step);
					jobSetAvalCount++;
				}
				unsJobIter++;
			}
			unschd.clear();
		}

		/************************************************************************************************************
		 * Modified Longest tail heuristic to take unreleased job with each iteration.								*
		 * NOTE: look in availableJobSteps list, update curTime to whatever head first job in available list has.	*
		 *		 This procedure done before we update available and release list based on curTime, to be less		*
		 *		 aggressive. Original setup.																		*
		 ************************************************************************************************************/
		if (!availableJobSteps.empty()) {
			step = availableJobSteps.top();
			if (step->curHead > curTime) {
				curTime = step->curHead;
			}
		}
	}
	if (jobSetAvalCount != mNumJobSteps)
		throw ERROR << "Not all job step can be scheduled, check precedence constraints.";
	int curSol = mOneMachDPData->calSolution(node);
	node->updateHeadInSol();
	// Reset curHead
	node->resetcurHead();

	node->mFeaSol = curSol;
	return curSol;
}

// Type 2: LLTH
int OneMachDPBounds::getUBMod2(OneMachDPNode* node)
{
	int curTime = 0;
	int maxTime = 0, temp = 0;
	int jobSetAvalCount = 0;
	int schdJobCount = 0;
	int jobIndex, updtIndex;
	JobStep* step;
	JobStep* prev;
	JobStep* tempStep;
	list<JobStep*> unschd;
	// availableJobSteps stores job steps that don't have unscheduled predecessor, ordered in ascending by head value
	minHeadQueue availableJobSteps(stepComp(3));
	// releaseJobSteps stores job steps that is not only available, but is released, ordered in descending by tail value
	maxTailQueue releaseJobSteps(stepComp(2));
	list<JobStep>::iterator iter = mOneMachDPData->mJobsData.begin();
	list<JobStep>::iterator end = mOneMachDPData->mJobsData.end();

	bool done = false;
	// Initialize: 1. move jobs to available pool if it does not have any predecessors
	//			   2. set job step curHead to head
	while (iter != end) {
		step = &*iter;
		jobIndex = step->jobIndex;
		step->curHead = step->head;
		if (node->mAllPreds[jobIndex].empty()) {
			availableJobSteps.push(step);
			jobSetAvalCount++;
		}
		iter++;
	}
	node->mJobScheduled.clear();
	node->mJobScheduled.resize(mNumJobSteps, -1);
	node->mSolPath.clear();
	// Sanity check. The first job should always be able to be added to queue
	step = availableJobSteps.top();
	if (curTime < step->curHead) curTime = step->curHead;

	while (schdJobCount < mNumJobSteps)
	{
		/************************************************************************************************************
		 * 1. Among all available jobs, find the ones that are released. Push into released job step poll.			*
		 *	  If none, move curTime to the starting time of the first job in available list and try again.          *
		 ************************************************************************************************************/
		while (!availableJobSteps.empty()) {
			step = availableJobSteps.top();
			if (step->curHead <= curTime) {
				releaseJobSteps.push(step);
				availableJobSteps.pop();
			}
			else if (releaseJobSteps.empty()) {
				curTime = step->curHead;
				releaseJobSteps.push(step);
				availableJobSteps.pop();
			}
			else
				break;
		}

		/************************************************************************************************************
		 * Modified Longest tail heuristic to take unreleased job with each iteration.								*
		 * NOTE: LLTH setup.																						*
		 ************************************************************************************************************/
		if (!availableJobSteps.empty() && !releaseJobSteps.empty()) {
			step = availableJobSteps.top();
			tempStep = releaseJobSteps.top();
			if (step->curHead > curTime && step->tail > tempStep->tail) {
				curTime = step->curHead;
			}
		}

		/************************************************************************************************************
		 * 2. Get the next job to schedule. Note that curTime should not be smaller than the head of step because   *
		 *	  previous section should have already updated curTime properly.                                        *
		 ************************************************************************************************************/
		step = releaseJobSteps.top();						// The step to schedule next
		node->mJobScheduled[step->jobIndex] = curTime;		// Record starting time of job step
		curTime += step->body;
		if (curTime + step->tail > maxTime) {
			maxTime = curTime + step->tail;
		}

		/************************************************************************************************************
		 * 3. Update the release date of successors of the job being scheduled, if necessary.                       *
		 ************************************************************************************************************/
		jobIndex = step->jobIndex;
		for (auto succJob = node->mAllSuccs[jobIndex].begin();
			succJob != node->mAllSuccs[jobIndex].end(); succJob++) {
			temp = (*succJob)->delay + curTime;
			if ((*succJob)->to->curHead < temp) {
				(*succJob)->to->curHead = temp;
				updtIndex = (*succJob)->to->jobIndex;
			}
			unschd.push_back((*succJob)->to);
		}
		/************************************************************************************************************
		 * 4. Put the job in existing schedule, keep track of scheduled jobs, and clean up.                         *
		 ************************************************************************************************************/
		node->mSolPath.push_back(step);
		schdJobCount++;
		releaseJobSteps.pop();
		prev = step;
		/************************************************************************************************************
		 * 5. For all the unscheduled jobs, the ones whoes predecessors have all been scheduled is put				*
		 *	  in a priority queue so they are sorted by head.														*
		 ************************************************************************************************************/
		if (!unschd.empty()) {
			auto unsJobIter = unschd.begin();
			while (unsJobIter != unschd.end()) {
				step = *unsJobIter;
				jobIndex = step->jobIndex;
				bool predDone = true;
				// Check the predecessor list for each unscheduled jobs to see if all predecessor scheduled
				for (auto predJob = node->mAllPreds[jobIndex].begin();
					predJob != node->mAllPreds[jobIndex].end(); predJob++) {
					int ind = (*predJob)->from->jobIndex;
					if (node->mJobScheduled[ind] == -1) {
						predDone = false;
						break;
					}
				}
				if (predDone) {
					availableJobSteps.push(step);
					jobSetAvalCount++;
				}
				unsJobIter++;
			}
			unschd.clear();
		}
	}
	if (jobSetAvalCount != mNumJobSteps)
		throw ERROR << "Not all job step can be scheduled, check precedence constraints.";
	int curSol = mOneMachDPData->calSolution(node);
	node->updateHeadInSol();
	// Reset curHead
	node->resetcurHead();

	node->mFeaSol = curSol;
	return curSol;
}

// Type 3
int OneMachDPBounds::getUBMod3(OneMachDPNode* node)
{
	int curTime = 0;
	int maxTime = 0, temp = 0;
	int jobSetAvalCount = 0;
	int schdJobCount = 0;
	int jobIndex, updtIndex;
	JobStep* step;
	JobStep* prev;
	JobStep* tempStep;
	list<JobStep*> unschd;
	// availableJobSteps stores job steps that don't have unscheduled predecessor, ordered in ascending by head value
	minHeadQueue availableJobSteps(stepComp(3));
	// releaseJobSteps stores job steps that is not only available, but is released, ordered in descending by tail value
	maxTailQueue releaseJobSteps(stepComp(2));
	list<JobStep>::iterator iter = mOneMachDPData->mJobsData.begin();
	list<JobStep>::iterator end = mOneMachDPData->mJobsData.end();

	bool done = false;
	// Initialize: 1. move jobs to available pool if it does not have any predecessors
	//			   2. set job step curHead to head
	while (iter != end) {
		step = &*iter;
		jobIndex = step->jobIndex;
		step->curHead = step->head;
		if (node->mAllPreds[jobIndex].empty()) {
			availableJobSteps.push(step);
			jobSetAvalCount++;
		}
		iter++;
	}
	node->mJobScheduled.clear();
	node->mJobScheduled.resize(mNumJobSteps, -1);
	node->mSolPath.clear();
	// Sanity check. The first job should always be able to be added to queue
	step = availableJobSteps.top();
	if (curTime < step->curHead) curTime = step->curHead;

	while (schdJobCount < mNumJobSteps)
	{
		/************************************************************************************************************
		 * 1. Among all available jobs, find the ones that are released. Push into released job step poll.			*
		 *	  If none, move curTime to the starting time of the first job in available list and try again.          *
		 ************************************************************************************************************/
		while (!availableJobSteps.empty()) {
			step = availableJobSteps.top();
			if (step->curHead <= curTime) {
				releaseJobSteps.push(step);
				availableJobSteps.pop();
			} else if (releaseJobSteps.empty()) {
				curTime = step->curHead;
				releaseJobSteps.push(step);
				availableJobSteps.pop();
			} else
				break;
		}

		/************************************************************************************************************
		 * Modified Longest tail heuristic to take unreleased job with each iteration.								*
		 * NOTE: Similar to Type 1, but wait until update available and release list to update time					*
		 ************************************************************************************************************/
		if (!availableJobSteps.empty()) {
			tempStep = availableJobSteps.top();
			temp = tempStep->curHead;
			while (tempStep->curHead <= temp) {
				releaseJobSteps.push(tempStep);
				availableJobSteps.pop();
				if (availableJobSteps.empty()) {
					break;
				}
				tempStep = availableJobSteps.top();
			}
			curTime = temp;
		}

		/************************************************************************************************************
		 * 2. Get the next job to schedule. Note that curTime should not be smaller than the head of step because   *
		 *	  previous section should have already updated curTime properly.                                        *
		 ************************************************************************************************************/
		step = releaseJobSteps.top();						// The step to schedule next
		node->mJobScheduled[step->jobIndex] = curTime;		// Record starting time of job step
		curTime += step->body;
		if (curTime + step->tail > maxTime) {
			maxTime = curTime + step->tail;
		}

		/************************************************************************************************************
		 * 3. Update the release date of successors of the job being scheduled, if necessary.                       *
		 ************************************************************************************************************/
		jobIndex = step->jobIndex;
		for (auto succJob = node->mAllSuccs[jobIndex].begin();
			succJob != node->mAllSuccs[jobIndex].end(); succJob++) {
			temp = (*succJob)->delay + curTime;
			if ((*succJob)->to->curHead < temp) {
				(*succJob)->to->curHead = temp;
				updtIndex = (*succJob)->to->jobIndex;
			}
			unschd.push_back((*succJob)->to);
		}
		/************************************************************************************************************
		 * 4. Put the job in existing schedule, keep track of scheduled jobs, and clean up.                         *
		 ************************************************************************************************************/
		node->mSolPath.push_back(step);
		schdJobCount++;
		releaseJobSteps.pop();
		prev = step;
		/************************************************************************************************************
		 * 5. For all the unscheduled jobs, the ones whoes predecessors have all been scheduled is put				*
		 *	  in a priority queue so they are sorted by head.														*
		 ************************************************************************************************************/
		if (!unschd.empty()) {
			auto unsJobIter = unschd.begin();
			while (unsJobIter != unschd.end()) {
				step = *unsJobIter;
				jobIndex = step->jobIndex;
				bool predDone = true;
				// Check the predecessor list for each unscheduled jobs to see if all predecessor scheduled
				for (auto predJob = node->mAllPreds[jobIndex].begin();
					predJob != node->mAllPreds[jobIndex].end(); predJob++) {
					int ind = (*predJob)->from->jobIndex;
					if (node->mJobScheduled[ind] == -1) {
						predDone = false;
						break;
					}
				}
				if (predDone) {
					availableJobSteps.push(step);
					jobSetAvalCount++;
				}
				unsJobIter++;
			}
			unschd.clear();
		}
	}
	if (jobSetAvalCount != mNumJobSteps)
		throw ERROR << "Not all job step can be scheduled, check precedence constraints.";
	int curSol = mOneMachDPData->calSolution(node);
	node->updateHeadInSol();
	// Reset curHead
	node->resetcurHead();

	node->mFeaSol = curSol;
	return curSol;
}