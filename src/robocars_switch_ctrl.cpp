/**
 * @file offb_raw_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 source $src_path/Tools/setup_gazebo.bash ${src_path} ${build_path}

 gzserver --verbose ${src_path}/Tools/sitl_gazebo/worlds/${model}.world &
 */
#include <tinyfsm.hpp>
#include <ros/ros.h>
#include <stdio.h>
#include <algorithm> 
#include <cmath>

#include <robocars_msgs/robocars_switch.h>
#include <robocars_msgs/robocars_radio_channels.h>
#include <robocars_msgs/robocars_brain_state.h>

#include <robocars_switch_ctrl.hpp>

RosInterface * ri;

static int loop_hz;

class onRunningMode;
class onIdle;
class onManualDriving;
class onAutonomousDriving;

class onRunningMode
: public RobocarsStateMachine
{
    public:
        onRunningMode() : RobocarsStateMachine("onRunningMode"),__tick_count(0) {};
        onRunningMode(const char * subStateName) : RobocarsStateMachine(subStateName),__tick_count(0) {};


    protected:

        uint32_t __tick_count;
        
        void entry(void) override {
            RobocarsStateMachine::entry();
        };

        void react(ManualDrivingEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react( AutonomousDrivingEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react( IdleStatusEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react (RadioChannelEvent const & e) override {
            ri->publishSwitch(e.ch2_value, e.ch4_value); 
        }

        void react(TickEvent const & e) override {
            __tick_count++;
            if ((__tick_count%loop_hz)==0) {
                //Update param each second
                ri->updateParam(); 
            }
        };

};

class onIdle
: public onRunningMode
{
    public:
        onIdle() : onRunningMode("onArm") {};

    private:

        void entry(void) override {
            onRunningMode::entry();
        };
  
        void react(ManualDrivingEvent const & e) override { 
            onRunningMode::react(e);
            transit<onManualDriving>();
        };

        void react(TickEvent const & e) override {
            onRunningMode::react(e);
        };

};

class onManualDriving
: public onRunningMode
{
    public:
        onManualDriving() : onRunningMode("onManualDriving") {};

    private:

        void entry(void) override {
            onRunningMode::entry();
        };

        void react (AutonomousDrivingEvent const & e) override {
            onRunningMode::react(e);
            transit<onAutonomousDriving>();
        }

        void react(IdleStatusEvent const & e) override { 
            onRunningMode::react(e);
            transit<onIdle>();
        };

        void react (TickEvent const & e) override {
            onRunningMode::react(e);
        };

};

class onAutonomousDriving
: public onRunningMode
{
    public:
        onAutonomousDriving() : onRunningMode("onAutonomousDriving") {};

    protected:

        virtual void entry(void) { 
            onRunningMode::entry();
        };  

        virtual void react(TickEvent                      const & e) override { 
            onRunningMode::react(e);
        };

        virtual void react(IdleStatusEvent                 const & e) override { 
            onRunningMode::react(e);
            transit<onIdle>();
        };

        virtual void react(ManualDrivingEvent                     const & e) override { 
            onRunningMode::react(e);
            transit<onManualDriving>();
        };

};

FSM_INITIAL_STATE(RobocarsStateMachine, onIdle);

u_int8_t channel2Lane (u_int32_t channelValue) {
    if ((channelValue)<600) {
        return robocars_msgs::robocars_switch::SWITCH_LANE_0;
    }
    if ((channelValue)>1550) {
        return robocars_msgs::robocars_switch::SWITCH_LANE_2;
    }
    return robocars_msgs::robocars_switch::SWITCH_LANE_1;
}


uint32_t mapRange(uint32_t in1,uint32_t in2,uint32_t out1,uint32_t out2,uint32_t value)
{
  if (value<in1) {value=in1;}
  if (value>in2) {value=in2;}
  return out1 + ((value-in1)*(out2-out1))/(in2-in1);
}

_Float32 mapRange(_Float32 in1,_Float32 in2,_Float32 out1,_Float32 out2,_Float32 value)
{
  if (value<in1) {value=in1;}
  if (value>in2) {value=in2;}
  return out1 + ((value-in1)*(out2-out1))/(in2-in1);
}

void RosInterface::initParam() {
    if (!nh.hasParam("loop_hz")) {
        nh.setParam ("loop_hz", 30);       
    }
}
void RosInterface::updateParam() {
    nh.getParam("loop_hz", loop_hz);
}

void RosInterface::initPub () {
    switch_pub = nh.advertise<robocars_msgs::robocars_switch>("/switch", 10);
}

void RosInterface::initSub () {
    channels_sub = nh.subscribe<robocars_msgs::robocars_radio_channels>("/radio_channels", 1, &RosInterface::channels_msg_cb, this);
    state_sub = nh.subscribe<robocars_msgs::robocars_brain_state>("/robocars_brain_state", 1, &RosInterface::state_msg_cb, this);
}

void RosInterface::channels_msg_cb(const robocars_msgs::robocars_radio_channels::ConstPtr& msg){    
    send_event(RadioChannelEvent(msg->ch2,msg->ch4));
}

void RosInterface::state_msg_cb(const robocars_msgs::robocars_brain_state::ConstPtr& msg) {
    static u_int32_t last_state = -1;
    if (msg->state != last_state) {
        switch (msg->state) {
            case robocars_msgs::robocars_brain_state::BRAIN_STATE_IDLE:
                send_event(IdleStatusEvent());        
            break;
            case robocars_msgs::robocars_brain_state::BRAIN_STATE_MANUAL_DRIVING:
                send_event(ManualDrivingEvent());        
            break;
            case robocars_msgs::robocars_brain_state::BRAIN_STATE_AUTONOMOUS_DRIVING:
                send_event(AutonomousDrivingEvent());        
            break;
        }
        last_state=msg->state;
    }
    
}

void RosInterface::publishSwitch (uint32_t ch2_value, uint32_t ch4_value) {

    robocars_msgs::robocars_switch switchMsg;

    switchMsg.header.stamp = ros::Time::now();
    switchMsg.header.seq=1;
    switchMsg.header.frame_id = "mainThrottling";
    switchMsg.lane = channel2Lane(ch4_value);

    switch_pub.publish(switchMsg);
}


int main(int argc, char **argv)
{
    int loopCnt=0;
    ros::init(argc, argv, "robocars_switch_ctrl_fsm");

    ri = new RosInterface;

    ri->initPub();
    fsm_list::start();
    ri->initSub();
    ROS_INFO("Switch Ctrl: Starting");

    // wait for FCU connection
    ros::Rate rate(loop_hz);
    while(ros::ok()){
        ros::spinOnce();
        send_event (TickEvent());
        rate.sleep();
    }
}

