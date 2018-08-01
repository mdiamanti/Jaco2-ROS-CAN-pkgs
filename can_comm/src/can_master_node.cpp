#include <ros/ros.h>
#include <std_msgs/String.h>
#include <can_msgs/Frame.h>
#include <ros/transport_hints.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_bridge/socketcan_to_topic.h>
#include <string>
#include <iostream>
#include <sstream>

using namespace std;


class FrameListener
{
private:
    size_t frameNum;
    can_msgs::Frame canFrame[7];
    bool receivedFrame;
public:
    void setFrameNum()
    {
	frameNum = 0;
    }

    size_t getFrameNum()
    {
        return frameNum;
    }

    void setReceivedFrame()
    {
        receivedFrame = false;
    }

    bool getReceivedFrame()
    {
	return receivedFrame;
    }

    can_msgs::Frame* getCanFrame()
    {
	return canFrame;
    }
	
    void frameCallBack(const can_msgs::Frame& f)
    {
        canFrame[frameNum] = f;
        ROS_INFO("CAN frame [%zu] arrived from PLC.", frameNum);
        frameNum++;
        if (frameNum == 7) receivedFrame = true;
    }
};

class StateListener
{
private:
    std_msgs::String state;
    bool receivedState;
public:
    void setReceivedState()
    {
        receivedState = false;
    }

    std_msgs::String getState()
    {
        return state;
    }

    bool getReceivedState()
    {
        return receivedState;
    }

    void stateCallBack(const std_msgs::String& msg)
    {
        state = msg;
        receivedState = true;
    }
};


// Method that converts a CAN frame to a string
string frame2string(const can_msgs::Frame& f)
{
    stringstream iss;
    string s;
	
    union{
        double myDouble;
        uint8_t myData[sizeof(double)];
    } convert;

    for (size_t i = 0; i < sizeof(double); ++i){
        convert.myData[i] = f.data[i];
    }
    
    iss << convert.myDouble;
    iss >> s;
    
    return s;
}


// Method that converts a string to a CAN frame
can_msgs::Frame string2frame(const string s)
{
    stringstream iss;
    double d;

    union{
        double myDouble;
        uint8_t myData[sizeof(double)];
    } convert;

    can_msgs::Frame f;
    f.is_extended = false;
    f.is_rtr = false;
    f.is_error = false;
    f.id = 0x123;
    f.dlc = 8;

    iss << s;
    iss >> d;
    convert.myDouble = d;
    cout << d << endl;

    for (size_t i = 0; i < sizeof(double); ++i){
        f.data[i] = convert.myData[i];
    }

    return f;
}


int main(int argc, char **argv)
{
    //Initialize ROS node
    ros::init(argc,argv, "can_master_node");

    // Create a handle to this process' node
    ros::NodeHandle nh;
	
    // Create publisher and subscriber objects
    FrameListener frListener;
    ros::Subscriber frameSub = nh.subscribe("/received_messages",100, &FrameListener::frameCallBack, &frListener, ros::TransportHints().tcpNoDelay());
    ros::Publisher posPub = nh.advertise<std_msgs::String>("/joint_positions_from_plc", 100);

    StateListener listener;
    ros::Subscriber stateSub = nh.subscribe("/current_state_to_plc", 100, &StateListener::stateCallBack, &listener, ros::TransportHints().tcpNoDelay());
    ros::Publisher framePub = nh.advertise<can_msgs::Frame>("/sent_messages", 100);

    // Declare variables
    string s;
    stringstream ss;
    std_msgs::String positions;
    can_msgs::Frame *frames;
    can_msgs::Frame stateFrame;

    while(ros::ok()){
	    // Subscribe to "/received_messages" topic so as to receive 7 CAN frames
        frListener.setReceivedFrame();
        frListener.setFrameNum();
        ROS_INFO("Waiting for CAN messages to arrive");
        while(!frListener.getReceivedFrame()){
            ros::spinOnce();
        }
        frames = frListener.getCanFrame();

	    // Convert each received CAN frame to a string and then concatenate all strings to one
        for (size_t i = 0; i < 7; ++i){
            if (i == 0){
                s = frame2string(frames[i]);
	        }else{
	            s = s + ' ' + frame2string(frames[i]);
	        }
        } 

        // Publish the joint position message (above created string) on "/joint_positions_from_plc" topic
        ss.str(s);
        positions.data = ss.str();
	    while (posPub.getNumSubscribers() == 0){
	        ros::spinOnce();
        }
        ROS_INFO("Got subscriber");
        posPub.publish(positions);
        ROS_INFO("The following joint position message is just published on /joint_positions_from_plc topic\n%s", positions.data.c_str());

        // Subscribe to "/current_state_to_plc" topic, so as to read the current joint state
        listener.setReceivedState();
        ROS_INFO("Waiting for current joint state to arrive from joint_trajectory_client node");
        while(!listener.getReceivedState()){
            ros::spinOnce();
        }
        ROS_INFO("Current joint state just arrived\n%s", listener.getState().data.c_str());
        ss.str(listener.getState().data);

        // Publish current joint state message on "/sent_messages" topic, so as to send it back to PLC 
        while (framePub.getNumSubscribers() == 0){
            ros::spinOnce();
        }
        ROS_INFO("Got subscriber");

        for (string token; getline(ss, token, ' '); ){
            stateFrame = string2frame(token);  // convert each string to a can frame
            stateFrame.header.frame_id = "0";  // "0" for no frame
            stateFrame.header.stamp = ros::Time::now();
            framePub.publish(stateFrame);
            ROS_INFO_STREAM(stateFrame);
            ros::Duration(0.5).sleep();  // sleep for half a second
        }
        ss.clear(); // clear flags of stringstream so as to reuse it

    }

    return 0;
}
