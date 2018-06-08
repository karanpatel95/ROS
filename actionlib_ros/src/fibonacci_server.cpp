#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_ros/FibonacciAction.h>

class FibonacciAction{

protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<actionlib_ros::FibonacciAction> as_;
    std::string action_name_;

    actionlib_ros::FibonacciFeedback feedback_;
    actionlib_ros::FibonacciResult result_;

public:
    FibonacciAction(std::string name) : action_name_(name),
        as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false)
    {
        as_.start();
    }

    ~FibonacciAction(void)
    {

    }

    void executeCB(actionlib_ros::FibonacciGoalConstPtr goal){

        ros::Rate r(1);
        bool success = true;

        feedback_.sequence.clear();
        feedback_.sequence.push_back(0);
        feedback_.sequence.push_back(1);

        // publish info
        ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

        // start executing the action
        for(int i = 0; i < goal->order; i++){
            // check that preempt has not been requested by the client
            if(as_.isPreemptRequested() || !ros::ok()){
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }
            feedback_.sequence.push_back(feedback_.sequence[i+1] + feedback_.sequence[i]);
            // publish the feedback
            as_.publishFeedback(feedback_);
            r.sleep();
        }

        if(success){
            result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }
};

int main(int argc, char** argv){

   ros::init(argc, argv, "fibonnaci");

   FibonacciAction fibonnaci("fibonnaci");
   ros::spin(); 

   return 0;
}
