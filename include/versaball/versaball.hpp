#ifndef VERSABALL_H
#define VERSABALL_H

#include <list>

#include <ros/ros.h>
// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <versaball/versaballConfig.h>
// Standard messages
#include <std_srvs/Trigger.h>

namespace versaball
{
    enum versaball_state{
        neutral,
        soft,
        jammed
    };

    /** An instance of this structure represents one of the four actuators (pump,
        valve, each for pressure or void).

        The index refers to the actual index on the Phidgets relay board, also
        used for the phidgets_ik services.

        The name is used to give contextual information in debug messages.
    **/
    struct effector_t{
        effector_t(std::string name, uint16_t index):
            name(name), index(index)
        {}
        std::string name;
        uint16_t index;
    };

    /** An "action" represents a change in the state (active, inactive) of an
        actuator (pump or valve), after some given time.

        We use lists of actions to represent, for instance, the transition
        between a soft (inflated, before grasp) Versaball and a jammed Versaball.
    **/
    struct action_t{
        action_t(ros::Duration instant, bool output_state, effector_t &effector):
            offset(offset), output_state(output_state), effector(effector)
        {}
        ros::Duration offset;
        bool output_state;
        effector_t &effector;
    };

    /**

        Note: According to [1], a service cannot be simultaneously called more
            than once
            [1]: http://answers.ros.org/question/11544/calling-a-ros-service-from-several-nodes-at-the-same-time/
    **/
    class VersaballNode
    {
    public:
        VersaballNode();

        /** Wait for the needed services to be advertised and in turn advertise ours,
            i.e. "grasp" and "release" in the private namespace.

            @return true if succeeded to advertise the services, and false otherwise
        **/
        bool advertise_services();

        // Give current state of the Versaball
        versaball_state state();
        const std::string state_str();

    private:
        void _hardware_setup();

        /** Actions requested to the Versaball.

            Transition table:
            | start state  |   action         |   goal_state |
            |--------------|------------------|--------------|
            | neutral      |   prepare_grasp  |   soft       |
            | soft         |   grasp          |   jammed     |
            | soft         |   release        |   neutral    |
            | jammed       |   release        |   neutral    |

            @return true if this action is allowed in the current state, false
                otherwise
        **/
        bool prepare_grasp();
        bool grasp();
        bool release();

        void _do_transition(std::list<action_t> actions_list);
        bool _execute_timed_action(const action_t& action, const ros::Duration& now);
        bool _set_phidgets_state(uint8_t index, uint16_t state);

        bool prepare_grasp_callback(std_srvs::Trigger::Request &req,
            std_srvs::Trigger::Response &res);
        bool grasp_callback(std_srvs::Trigger::Request &req,
            std_srvs::Trigger::Response &res);
        bool release_callback(std_srvs::Trigger::Request &req,
            std_srvs::Trigger::Response &res);
        bool state_callback();

        void dynamic_reconfigure_cb(versaball::versaballConfig &config, uint32_t level);

        versaball_state _current_state;

        // information about each effector and the related output
        // (of phidgets_interface_kit)
        effector_t _pressure_pump, _pressure_valve, _void_pump, _void_valve;

        // lists of actions needed for each transition between two states
        std::list<action_t> _prepare_grasp_a, _grasp_a, _release_grasp_a,
            _release_prepare_grasp_a;

        ros::NodeHandle _nh;;
        // Handles for the service we advertise; deletion will unadvertise the service
        ros::ServiceServer _prepare_grasp_service, _grasp_service, _release_service;

        // Variables used by dynamic reconfigure
        dynamic_reconfigure::Server<versaball::versaballConfig> _dynamic_reconfigure_server;
        dynamic_reconfigure::Server<versaball::versaballConfig>::CallbackType _dynamic_reconfigure_cb_t;
    };
}

#endif // VERSABALL_H