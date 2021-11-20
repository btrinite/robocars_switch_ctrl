/**
 * @file robocars_switch_ctrl.hpp
 * @brief convert discret radio channel value to three level value, usefull to mark image manually while driving for example.
 * 
 * Copyright (c) 2020 Benoit TRINITE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */

#include <tinyfsm.hpp>
#include <ros/ros.h>
#include <stdio.h>

struct BaseEvent : tinyfsm::Event
{
    public:
        BaseEvent(const char * evtName) : _evtName(evtName) {};
        const char * getEvtName() const { return _evtName; };
    private:
        const char *  _evtName;
};

struct TickEvent                    : BaseEvent { public: TickEvent() : BaseEvent("TickEvent") {}; };
struct IdleStatusEvent              : BaseEvent { public: IdleStatusEvent() : BaseEvent("IdleStatusEvent") {}; };
struct ManualDrivingEvent           : BaseEvent { public: ManualDrivingEvent() : BaseEvent("ManualDrivingEvent") {}; };
struct AutonomousDrivingEvent       : BaseEvent { public: AutonomousDrivingEvent() : BaseEvent("AutonomousDrivingEvent") {}; };
struct RadioChannelEvent            : BaseEvent { public: 
    RadioChannelEvent(const uint32_t ch2_value, const uint32_t ch4_value) : ch2_value(ch2_value),ch4_value(ch4_value), BaseEvent("RadioChannelEvent") {};
    uint32_t ch2_value; 
    uint32_t ch4_value; 
    };

class RobocarsStateMachine
: public tinyfsm::Fsm<RobocarsStateMachine>
{
    public:
        RobocarsStateMachine(const char * stateName) : _stateName(stateName), tinyfsm::Fsm<RobocarsStateMachine>::Fsm() { 
            ROS_INFO("Switch Ctrl StateMachine: State created: %s", _stateName);      
        };
        const char * getStateName() const { return _stateName; };

    public:
        /* default reaction for unhandled events */
        void react(BaseEvent const & ev) { 
            ROS_INFO("state %s: unexpected event %s reveived", getStateName(), ev.getEvtName());      
        };

        virtual void react(TickEvent                      const & e) { /*logEvent(e);*/ };
        virtual void react(IdleStatusEvent                const & e) { logEvent(e); };
        virtual void react(ManualDrivingEvent             const & e) { logEvent(e); };
        virtual void react(AutonomousDrivingEvent         const & e) { logEvent(e); };
        virtual void react(RadioChannelEvent              const & e) {  };

        virtual void entry(void) { 
            ROS_INFO("Switch Ctrl : State %s: entering", getStateName()); 
        };  
        void         exit(void)  { };  /* no exit actions */

    private:
        const char *  _stateName ="NoName";
        void logEvent(BaseEvent const & e) {
            ROS_INFO("State %s: event %s", getStateName(), e.getEvtName());
        }
};

typedef tinyfsm::FsmList<RobocarsStateMachine> fsm_list;

template<typename E>
void send_event(E const & event)
{
  fsm_list::template dispatch<E>(event);
}

class RosInterface
{
    public :
        RosInterface() {
            initParam();
            updateParam();
        };


        void initParam();
        void updateParam();
        void initPub();
        void initSub();

        void publishSwitch (uint32_t ch2_value, uint32_t ch4_value);

    private:

        void channels_msg_cb(const std_msgs::Int16MultiArray::ConstPtr& msg);
        void state_msg_cb(const robocars_msgs::robocars_brain_state::ConstPtr& msg);

        ros::NodeHandle nh;
        ros::Publisher annotation_pub;
        ros::Publisher switch_pub;
        ros::Subscriber channels_sub;
        ros::Subscriber state_sub;

};

