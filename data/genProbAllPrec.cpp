#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <chrono>
#include <vector>

using namespace std;

typedef chrono::high_resolution_clock myclock;

bool hasPrec(int from, int to, vector<vector<bool>> &predecessor_mat)
{
    if (from == to) return false;

    return predecessor_mat[to][from];
}

void updatePrec(int from, int to, vector<vector<bool>> &predecessor_mat)
{
    predecessor_mat[to][from] = true;
    for (int i = 0; i < predecessor_mat[from].size(); i++)
    {
        if (!predecessor_mat[to][i] && predecessor_mat[from][i])
            updatePrec(i,to,predecessor_mat);
    }
}

// class Prec_manage
// {
// public:
//     precManage(int size_inst)
//     {
//         m_prec_mat.resize(size_inst);
//         for (int i = 0; i < size_inst; i++)
//         {
//             m_prec_mat[i].resize(size_inst, 0);
//         }
//         m_size_inst = size_inst;
//     }

//     vector<vector<int>> m_prec_mat
//     int m_size_inst = 0;
// }

int main()
{
    myclock::time_point beginning = myclock::now();
    int head, body, tail;
    int bodyMax = 50;
    int coef = 15;
    int numJobs = 100;
    int htCoef = 50;
    int htMax;
    int offset = 1;
    int numProb = 1000;
    int curProb, precCount, precLength;
    char path[100];
    myclock::duration d;
    vector<vector<int>> hasPrecConstr;
    vector<vector<bool>> predecessors;
    vector<int> jobBodies;
    for (int prob = 6; prob <= 10; prob += 2) {
        hasPrecConstr.clear();
        hasPrecConstr.resize(numJobs);
        predecessors.clear();
        predecessors.resize(numJobs);

        for (int i = 0; i < numProb; i++) {
            jobBodies.clear();

            htMax = numJobs * bodyMax * coef / htCoef;
            d = myclock::now() - beginning;
            srand(d.count());

            char filename[100];
            sprintf(filename,"oneMach_J%d_B%d_C%d_HC%d_P%d_%d.txt",numJobs, bodyMax, coef, htCoef, prob, i);
            //sprintf(path,"omdpJ%dK%dP%dSAE/%s", numJobs, coef, prob, filename);
            sprintf(path,"omdpJ%dK%dP%dSAE/%s", numJobs, coef, prob, filename);
            printf("[%s]\n", filename);
            FILE* probFile = fopen(path,"w");
            fprintf(probFile, "%s\n", filename);
            fprintf(probFile, "%d\n", numJobs);

            for (int i = 0; i < numJobs; i++) {
                body = rand() % bodyMax + 1;
                head = rand() % htMax + 1;
                tail = rand() % htMax + 1;
                jobBodies.push_back(body);
                fprintf(probFile, "%d %d %d\n", head, body, tail);
            }

            precCount = 0;
            for (int i = 0; i < numJobs; i++) {
                hasPrecConstr[i].clear();
                hasPrecConstr[i].resize(numJobs,-1);
                predecessors[i].clear();
                predecessors[i].resize(numJobs, false);
            }
            for (int i = 0; i < numJobs; i++) {
                for (int j = 0; j < numJobs; j++) {
                    if (i == j) continue;
                    if (!hasPrec(j,i,predecessors)) {
                        curProb = rand() % 100;
                        if (curProb < prob) {
                            precCount++;
                            precLength = rand() % htMax + 1 - jobBodies[i];
                            hasPrecConstr[i][j] = (precLength > 0) ? precLength : precLength + jobBodies[i];
                            updatePrec(i,j,predecessors);
                            //printf("Current from %d to %d.\n", i, j);
                        }
                    }
                }
            }

            fprintf(probFile, "%d\n", precCount);
            for (int i = 0; i < numJobs; i++) {
                for (int j = 0; j < numJobs; j++) {
                    if (hasPrecConstr[i][j] > 0)
                        fprintf(probFile, "%d %d %d\n", i, j, hasPrecConstr[i][j]);
                }
            }

            fclose(probFile);
        }
    }
    return 0;
}
