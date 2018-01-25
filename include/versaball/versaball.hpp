#ifndef VERSABALL_H
#define VERSABALL_H

// Standard library
#include <list>

// ROS-related
#include <ros/ros.h>
// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <versaball/versaballConfig.h>
// Service specific to Versaball
#include <versaball/GetState.h>
// Standard services
#include <std_srvs/Trigger.h>

namespace versaball {
    enum versaball_state {
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
    struct effector_t {
        effector_t(std::string name, uint16_t index) : name(name), index(index)
        {
        }
        std::string name;
        uint16_t index;
    };

    /** An "action" represents a change in the state (active, inactive) of an
        actuator (pump or valve), after some given time.

        We use lists of actions to represent, for instance, the transition
        between a soft (inflated, before grasp) Versaball and a jammed Versaball.
    **/
    struct action_t {
        action_t(ros::Duration trigger, bool output_state, effector_t& effector)
            : trigger(trigger), output_state(output_state), effector(effector) {}
        ros::Duration trigger;
        bool output_state;
        effector_t& effector;
    };

    /**

        Transition table for the actions requested to the Versaball:
        | initial state |   action         |  final state |
        |---------------|------------------|--------------|
        | neutral       |   prepare_grasp  |   soft       |
        | soft          |   grasp          |   jammed     |
        | soft          |   release        |   neutral    |
        | jammed        |   release        |   neutral    |
        All other actions are forbidden. Requesting them will have no effect and
        the response will contain `false` and an explanatory message.

        Note: According to [1], a service cannot be simultaneously called more
            than once
            [1]: http://answers.ros.org/question/11544/calling-a-ros-service-from-several-nodes-at-the-same-time/
    **/
    class VersaballNode {
    public:
        VersaballNode();

        /** Wait for the needed services to be advertised and in turn advertise ours,
            i.e. "grasp" and "release" in the private namespace.

            @return true if succeeded to advertise the services, and false otherwise
        **/
        bool advertise_services();

        // Give current state of the Versaball
        versaball_state state();
        const std::string state_str(); // as a string

    private:
        // define the sub-actions needed for each transition in our modest state
        // machine
        void _hardware_setup();

        // executes all actions in the provided list, in their timed order,
        // performing the transition between two states
        void _do_transition(std::list<action_t> actions_list);
        // if the relative time is bigger than the trigger time of the action,
        // executes it
        // by "relative time", we mean relative to the start of the transition,
        // when _do_transition is called
        bool _execute_timed_action(const action_t& action,
            const ros::Duration& relative_time);
        // call a service of phidgets_interface_kit to set the state of one
        // of its relays
        bool _set_phidgets_state(uint8_t index, uint16_t state);

        // callbacks for the services offered by the Versaball
        bool _prepare_grasp_callback(std_srvs::Trigger::Request& req,
            std_srvs::Trigger::Response& res);
        bool _grasp_callback(std_srvs::Trigger::Request& req,
            std_srvs::Trigger::Response& res);
        bool _release_callback(std_srvs::Trigger::Request& req,
            std_srvs::Trigger::Response& res);
        bool _state_callback(GetState::Request& req, GetState::Response& res);

        // callback periodically pumping some air out of the Versaball, to
        // compensate for it's slight leaking
        void _keep_grasp_callback(const ros::TimerEvent& event);

        // callback for dynamic reconfigure (update the parameter values)
        void dynamic_reconfigure_cb(versaball::versaballConfig& config, uint32_t level);

        versaball_state _current_state;

        // information about each effector and the related output
        // (of phidgets_interface_kit)
        effector_t _pressure_pump, _pressure_valve, _void_pump, _void_valve;

        // lists of actions needed for each transition between two states
        std::list<action_t> _prepare_grasp_a, _grasp_a, _release_grasp_a,
            _release_prepare_grasp_a;

        ros::NodeHandle _nh;
        ;
        // handles for the service we advertise; deletion will unadvertise the
        // service
        ros::ServiceServer _prepare_grasp_service, _grasp_service,
            _release_service, _state_service;

        ros::ServiceClient _phidgets_client;

        // timings for the grasping state, when we regularly restart the void
        // system:    time between calls    how long we suck air
        ros::Duration _keep_grasp_period, _keep_grasp_duration;
        // handle for the timed grasp keeping method
        ros::Timer _keep_grasp_timer;

        // Variables used by dynamic reconfigure
        dynamic_reconfigure::Server<versaball::versaballConfig> _dynamic_reconfigure_server;
        dynamic_reconfigure::Server<versaball::versaballConfig>::CallbackType _dynamic_reconfigure_cb_t;
    };
} // namespace versaball

#endif // VERSABALL_H
