//
// Created by jieming on 29.11.20.
//

#ifndef PANDA_CONTROL_RRT_H
#define PANDA_CONTROL_RRT_H

#include "common.h"

typedef vector<double> Configuration;
typedef int Index;

class RRTNode{
private:
    Configuration config_;  // store a C-space point
    Index parentIndex_;
    vector<double> dofWeights_;  // coefficients to reweight resolution for each joint

public:
    RRTNode( const Configuration& config){
        config_ = config;
        parentIndex_ = -1;
     }

    // add parent node's index in NodeTree
    void addParent(Index parentIndex){
        parentIndex_ = parentIndex;
    }

    // Euclidean distance to sample point
    double distanceTo(const Configuration &sampleConfig){
        double distance = 0.0;
        for(uint i=0; i < config_.size(); ++i){
            distance +=  pow( (config_[i]-sampleConfig[i])*dofWeights_[i], 2);
        }
        return sqrt(distance);
    }

    Configuration getConfig(){
        return config_;
    }

    Index getParentIndex(){
        return parentIndex_;
    }

    bool isGoal(const Configuration &goalConfig){
        bool flag = true;
        for (unsigned int i = 0; i < config_.size(); ++i){
            if (fabs(config_[i] - goalConfig[i]) > 0.01){
                flag = false;
                break;
            }
        }
        return flag;
    }

};

class NodeTree
{
private:
    vector<RRTNode> nodes_; // nodes stored in RRT
    Configuration goalConfig_;
    bool goalConnected_;
    double stepSize_;
    vector<Configuration> path_; // path found
    int nSample_;
    int spaceDim_;  // dimension of C-space

public:
    bool init(  const double step_size, const Configuration &initConfig, const Configuration &goalConfig){
        goalConnected_ = false;
        stepSize_ = step_size;
        goalConfig_ = goalConfig;

        addNode(initConfig); // initialize a new tree by adding initConfig as the first node

        spaceDim_ = goalConfig.size();
        nSample_ = 0;

        return true;
    }

    int treeSize(){
        return nodes_.size();
    }

    // check if sample is in C_free
    bool inCollision(const Configuration &sampleConfig)
    {
        _robotPtr->SetActiveDOFValues(sampleConfig);

        if ( _envPtr->CheckCollision(_robotPtr) || _robotPtr->CheckSelfCollision() )
            return true;
        else
            return false;

    }

    // check if Euclidean distance between sample and NN is within step size
    bool withinStepSize(const Configuration &sampleConfig){
        return nodes_[nearestNodeIndex(sampleConfig)].distanceTo(sampleConfig) <= stepSize_ ;
    }

    // find nearest node from sample point
     Index nearestNodeIndex(const Configuration &sampleConfig){
        float minDist = 1E6;
        int minIndex = -1;

        for (uint i=0; i<nodes_.size(); ++i){
            if (nodes_[i].distanceTo(sampleConfig) < minDist){
                if (minDist <= 0.01){
                    minIndex = i;
                    break;
                }
                else{
                    minDist = nodes_[i].distanceTo(sampleConfig);
                    minIndex = i;
                }
            }
        }
        return minIndex;
    }

    // the EXTEND algorithm
    void addNode(  Configuration &sampleConfig){
        nSample_++;
        if (treeSize() == 0){
            RRTNode initNode(sampleConfig);
            nodes_.push_back(initNode);
         }
        else{
            // find NN to current sample point
            Index NNIndex = nearestNodeIndex(sampleConfig);
            int numSteps = nodes_[NNIndex].distanceTo(sampleConfig)/stepSize_;  // how many steps to add until distance to sample < step size

            // if sample is outside of step size
            if(!withinStepSize(sampleConfig)) {
                for (uint i = 0; i < spaceDim_; ++i){
                    sampleConfig[i] = nodes_[NNIndex].getConfig()[i] + (sampleConfig[i]-nodes_[NNIndex].getConfig()[i])*(1.0/numSteps*0.8) ;
                }
            }
            // sample is now within stepsize to NN
            // if sample is in C_free, add to tree directly; otherwise, do nothing
            if (!inCollision(sampleConfig)){
                RRTNode newNode(sampleConfig);
                newNode.addParent(NNIndex);
                nodes_.push_back(newNode);
                // after sample node is added into tree, check if it's goal
                // if sample is goal, planning succeeds
                if (newNode.isGoal(goalConfig_)){
                    goalConnected_ = true;
                    // construct path by backtracking sequence of parent nodes from goal to init
                    Index wayIndex = nodes_.size()-1;
                    while (wayIndex != -1){
                        path_.push_back(nodes_[wayIndex].getConfig());
                        wayIndex = nodes_[wayIndex].getParentIndex();
                    }
                }
            }

        }
    }

#if 0
    void smooth(vector<Configuration> &path){
        // cout << "original len: " << path.size() << endl;
        for (int i = 0; i < 200; ++i){
            // if less than 10 nodes on path, stop shortcutting
            if (path.size() <= 5)
                break;

            int sample1 = rand()%path.size();
            int sample2 = rand()%path.size();

            // if
            while ((sample1 == 0 && sample2 == 0) || abs(sample1-sample2 <= 1))
            {
                sample1 = rand()%path.size();
                sample2 = rand()%path.size();
            }

            int first = min(sample1, sample2);
            int second = max(sample1, sample2);

            RRTNode firstNode(_envPtr, path[first]);

            // if without one step, try each step
            if (firstNode.distanceTo(path[second]) > _stepSize)
            {
                int numSteps = firstNode.distanceTo(path[second])/_stepSize;  // how many steps to add until distance to sample < step size
                float newSamplePos = (numSteps*_stepSize)/firstNode.distanceTo(path[second]);  // position of furthest step on local path

                bool shortCutFailed = false;
                // add one node each step forward on local path
                for (int step = 1; step <= numSteps; ++step)
                {
                    // initialize step point
                    Configuration newStepConfig(_spaceDim);

                    // construct step point
                    for (int d = 0; d < _spaceDim; ++d)
                        newStepConfig[d] = path[first][d] + ((path[second][d]-path[first][d])*newSamplePos*step)/numSteps;

                    // if step in collision, discard connection
                    if (inCollision(newStepConfig))
                    {
                        shortCutFailed = true;
                        break;
                    }
                }

                // if all steps are collision-free, do shortcut
                if (shortCutFailed == false)
                    path.erase(path.begin()+first+1, path.begin()+second);

                // cout << "path len: " << path.size() << endl;

            }

        }

        // cout << "smoothed len: " << path.size() << endl;
    }
#endif

    void clear(){
        nodes_.clear();
        path_.clear();
    }

    bool goalConnected(){
        return goalConnected_;
    }

    vector<Configuration> getPath(){
        return path_;
    }

    void printStatistics(){
        cout << "sampled: " << nSample_ << endl;
        cout << "tree size: " << nodes_.size() << endl;
    }
};



#endif //PANDA_CONTROL_RRT_H
