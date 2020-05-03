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

        void channels_msg_cb(const robocars_msgs::robocars_radio_channels::ConstPtr& msg);
        void state_msg_cb(const robocars_msgs::robocars_brain_state::ConstPtr& msg);

        ros::NodeHandle nh;
        ros::Publisher switch_pub;
        ros::Subscriber channels_sub;
        ros::Subscriber state_sub;

};

