#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <sstream>

using namespace std;


class PositionListener
{
private:
    std_msgs::String jointPosition;
    bool receivedPos;
public:
    void setReceivedPos()
    {
        receivedPos = false;
    }

    std_msgs::String getJointPosition()
    {
	return jointPosition;
    }
	
    bool getReceivedPos()
    {
        return receivedPos;
    }

    void jointPositionCallback(const std_msgs::String& msg)
    {
	jointPosition = msg;
	receivedPos = true;
    }
};


class StateListener
{
private:
    sensor_msgs::JointState jointState;
    bool receivedJs;
public:
    void setReceivedJs()
    {
        receivedJs = false;
    }

    sensor_msgs::JointState getJointState() 
    {
	return jointState;
    }
	
    bool getReceivedJs()
    {
	return receivedJs;
    }

    void jointStateCallback(const sensor_msgs::JointState& msg)
    {
	jointState = msg;
	receivedJs = true;
    }
};


// Method that parses the string published on /joint_positions_from_plc topic
vector<double> messageParser(ros::NodeHandle& n)
{
    PositionListener posListener;
    posListener.setReceivedPos();
	 
    // Subscribe to /joint_positions_from_plc topic
    ros::Subscriber posSub = n.subscribe("/joint_positions_from_plc", 100, &PositionListener::jointPositionCallback, &posListener, ros::TransportHints().tcpNoDelay());
    ROS_INFO("Wait desired joint positions to arrive from PLC");
    while(!posListener.getReceivedPos()){
        ros::spinOnce();
    }
    ROS_INFO("Desired joint positions just arrived");
    ROS_INFO_STREAM(posListener.getJointPosition());
 
    istringstream iss(posListener.getJointPosition().data);

    posSub.shutdown(); // shut down subscriber

    return vector<double>{istream_iterator<double>(iss),istream_iterator<double>()};
}


// Method that returns the joint state of the arm published on /j2n6s300/joint_states topic
sensor_msgs::JointState getCurrentJointState(ros::NodeHandle& n)
{
    StateListener stListener;
    stListener.setReceivedJs();

    // Subscribe to /j2n6s300/joint_states topic
    ros::Subscriber jsSub = n.subscribe("/j2n6s300/joint_states", 100, &StateListener::jointStateCallback, &stListener, ros::TransportHints().tcpNoDelay());
    ROS_INFO("Wait until joint state arrives");
    while (!stListener.getReceivedJs()){
        ros::spinOnce();
    }
    ROS_INFO("Joint state just arrived");
    ROS_INFO_STREAM(stListener.getJointState());
	
    sensor_msgs::JointState js = stListener.getJointState();
    jsSub.shutdown(); // shut down subscriber
	
    return js; 	
}


int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "joint_trajectory_client");
	
    // Create a handle to this process' node
    ros::NodeHandle n;
  
    // Create Publisher object
    ros::Publisher statePub = n.advertise<std_msgs::String>("/current_state_to_plc", 100);

    // Create action client, true causes the client to spin its own thread
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/j2n6s300/effort_joint_trajectory_controller/follow_joint_trajectory", true);

    ROS_INFO("Wait for action server to start");
    ac.waitForServer();  // will wait for infinite time
    ROS_INFO("Action server started");
  
    // Declare variables
    vector<double> positionMsg, finalPosition;
    size_t pathLength;
    float timeOffset, timeStep;
    stringstream ss;
    std_msgs::String stateMsg;
    sensor_msgs::JointState currentState, finalState;	
    bool finished_before_timeout;

    // Create goal trajectory object
    control_msgs::FollowJointTrajectoryGoal goal;
 
    // Set joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("j2n6s300_joint_1");
    goal.trajectory.joint_names.push_back("j2n6s300_joint_2");
    goal.trajectory.joint_names.push_back("j2n6s300_joint_3");
    goal.trajectory.joint_names.push_back("j2n6s300_joint_4");
    goal.trajectory.joint_names.push_back("j2n6s300_joint_5");
    goal.trajectory.joint_names.push_back("j2n6s300_joint_6");
  
    while(ros::ok()){
	    ss.str(std::string()); // clear stringstream
        ss.clear();	
	
        // Receive joint position message from PLC
        positionMsg = messageParser(n);

        //Select mode, 0 refers to absolute joint movement and 1 refers to relative joint movement
        switch(int(positionMsg[0])){
        
        case(0): // absolute joint movement
            // Set number of waypoints for goal trajectory
            pathLength = 1;
            goal.trajectory.points.resize(pathLength);
            goal.trajectory.points[0].positions.resize(6);
            goal.trajectory.points[0].velocities.resize(6);

            for (size_t i = 0; i < 6; ++i){
                goal.trajectory.points[0].positions[i] =  positionMsg[i+1];
                goal.trajectory.points[0].velocities[i] = 0.0;
            }   
  
            // To be reached 10.0 second after starting along the trajectory
            goal.trajectory.points[0].time_from_start = ros::Duration(10.0);
            ROS_INFO("Absolute joint movement will be executed");        
            break;

        case(1): // relative joint movement
            // Set number of waypoints for goal trajectory
            pathLength = 2;
            goal.trajectory.points.resize(pathLength);

            timeOffset = 0;
            timeStep = 10;
			
            // Receive current (initial) joint state  
            currentState = getCurrentJointState(n);

            // Set positions and velocities of waypoints 
            for (size_t i = 0; i < pathLength; i++){
                goal.trajectory.points[i].positions.resize(6);
                goal.trajectory.points[i].velocities.resize(6);
                goal.trajectory.points[i].time_from_start = ros::Duration(timeOffset + timeStep * i);       
                for (size_t j = 0; j < 6; j++){
                    goal.trajectory.points[i].positions[j] = currentState.position[j];
                    goal.trajectory.points[i].velocities[j] = 0.0;
                }
            }
        
            for (size_t i = 0; i < 6; ++i){
               goal.trajectory.points[1].positions[i] += positionMsg[i+1];
            } 
            ROS_INFO("Relative joint movement will be executed");
            break;
       }
	
       // Start trajectory in 1 sec from now
       goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    
       // Send goal and start the trajectory
       ac.sendGoal(goal);
       ROS_INFO("Goal trajectory just started");  

       // Wait for action to return
       finished_before_timeout = ac.waitForResult(ros::Duration(20.0));
       if (finished_before_timeout){
           actionlib::SimpleClientGoalState state = ac.getState();
           ROS_INFO("Action finished: %s", state.toString().c_str());
       }else{
           ROS_WARN("Action did not finish before the time out");
       }    

       // Receive current (final) joint state   
       finalState = getCurrentJointState(n);
       finalPosition = finalState.position;

       // Concatenate current (final) state vector, so as to send it back to PLC
       for (size_t i = 0; i < 6; i++){
           if (i != 0) ss << " ";
           ss << finalPosition[i];
       }
       stateMsg.data = ss.str();

       // Publish current joint state on /current_state_to_plc topic
       while (statePub.getNumSubscribers() == 0){
           ros::spinOnce();
       }
       ROS_INFO("Got subscriber");
       statePub.publish(stateMsg);
       ROS_INFO("The following current joint state message is just published on /current_state_to_plc topic\n%s", stateMsg.data.c_str()); 
    }
	
    return 0;
}
