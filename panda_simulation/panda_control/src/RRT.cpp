//
// Created by jieming on 29.11.20.
//

#include "RRT.h"

class RRT{
private:
    // search tree
    NodeTree tree_;

    // planner parameters
    double goalFrequency_;
    int maxIteration_;
    double stepSize_;
    int spaceDim_;

    // init and goal
    Configuration initConfig_;
    Configuration goalConfig_;

    // DOF limits
    Configuration lowerDOFLimits_;
    Configuration upperDOFLimits_;

    // path found
    vector<Configuration> myPath_;

public:
    RRT():  initConfig_(7), goalConfig_(7){}

    void setPlannerPar(){
//        goalFrequency_ = ;
        maxIteration_ = 5000;
        stepSize_ = 0.2;
        spaceDim_ = 7;
     }

    void setInitialConfig(const vector<double>& input ){
         for(int i = 0; i < spaceDim_; ++i){
            initConfig_[i] = input[i];
        }
     }

    void setGoalConfig(vector<double> input){
         for(int i = 0; i < spaceDim_; ++i){
            goalConfig_[i] = input[i];
        }
    }

    void init(std::ostream& sout, std::istream& sinput){
        //Initialize the tree
        tree_.init(stepSize_, initConfig_, goalConfig_);

        //Set Robot limits
        lowerDOFLimits_[4] = -3.1416;
        lowerDOFLimits_[6] = -3.1416;
        upperDOFLimits_[4] = 3.1416;
        upperDOFLimits_[6] = 3.1416;

    }

    bool run(std::ostream& sout, std::istream& sinput){
        if(pathFound()){
            sout << "success";
        }
        else{
            sout << "failed";
        }

        tree_.printStatistics();
        return true;
    }

    void getFingerPath(){
        for (unsigned int i = 0; i < myPath_.size(); ++i){
            RaveVector<double> Trans = robot->GetManipulators()[6]->GetEndEffectorTransform().trans;
        }
    }

#if 0
    bool smoothPath(std::ostream& sout, std::istream& sinput){
        _tree.smooth(_myPath);
        return true;
    }
#endif

    void clearTree(){
        tree_.clear();
        myPath_.clear();
     }

    // run RRT for maxIteration times and report if path is found
    bool pathFound(){
        bool success_flag=false;
        for (uint i=0; i < maxIteration_; ++i){
             if (tree_.goalConnected()){
                myPath_ = tree_.getPath();
                reverse(myPath_.begin(), myPath_.end());
                success_flag = true;
                break;
            }
             Configuration sampleConfig = sampleNewConfig();
             tree_.addNode(sampleConfig);
        }

         return success_flag;
    }

    Configuration sampleNewConfig(){
        srand(1);
        Configuration sampleConfig(spaceDim_);

        for(int i = 0; i<spaceDim_; ++i){
             sampleConfig[i] = lowerDOFLimits_[i] +  (upperDOFLimits_[i]-lowerDOFLimits_[i])*  (rand()/double(RAND_MAX)) ;
         }

        return sampleConfig;
    }

#if 0
    vector<string> tokenize(std::istream& sinput){
        string input;
        sinput >> input;

        cout << input << endl;

        vector<string> vals;
        boost::char_separator<char> sep(",");
        boost::tokenizer<boost::char_separator<char> > tok(input, sep);
        for(boost::tokenizer<boost::char_separator<char> >::iterator iter = tok.begin(); iter != tok.end(); ++iter)
        {
            vals.push_back(*iter);
            cout << *iter << endl;
        }

        return vals;
    }

        Configuration sampleNewConfig(){
        if ((float) rand()/(float) RAND_MAX < goalFrequency_){
            return goalConfig_;
        }
        else{
            Configuration sampleConfig(spaceDim_);

            for(int i = 0; i<spaceDim_; ++i){
                sampleConfig[i] = lowerDOFLimits_[i] + (float) rand()*(upperDOFLimits_[i]-lowerDOFLimits_[i])/(float) RAND_MAX;
             }

            return sampleConfig;
        }
    }
#endif

};

