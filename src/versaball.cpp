#include <phidgets_interface_kit/setStates.h> // TODO: remove this include
#include <phidgets_interface_kit/setState.h>

#include <versaball/versaball.hpp>

namespace versaball
{
    VersaballNode::VersaballNode():
        _nh("~"),
        _pressure_pump("pressure pump", 1),
        _pressure_valve("pressure valve", 2),
        _void_pump("void pump", 0),
        _void_valve("void valve", 3),
        _keep_grasp_period(),
        _keep_grasp_duration()
    {
        // Wait until ROS gives time information
        while(ros::Time::now().toSec() == 0)
        {
            // Wait a duration of 10 milliseconds
            ros::Duration d = ros::Duration(0.010);
            d.sleep();
        }

        // Fill the actions definitions for each transition in our state machine
        _hardware_setup();

        // Set the current state to the default value
        _current_state = neutral;

        // Set the callback for dynamic reconfiguration of the node's parameters
        _dynamic_reconfigure_cb_t = boost::bind(
            &VersaballNode::dynamic_reconfigure_cb,
            this,
            _1,
            _2);
        _dynamic_reconfigure_server.setCallback(_dynamic_reconfigure_cb_t);
    }

    bool VersaballNode::advertise_services()
    {
        bool success = false;
        std::string phidgets_service = "phidgets_ik/set_state";

        ROS_INFO_STREAM("Waiting for \""<<phidgets_service<<"\" service"
            "(2 seconds).");
        if (ros::service::waitForService(phidgets_service, 2000))
        {

            _prepare_grasp_service = _nh.advertiseService("prepare_grasp",
                &VersaballNode::_prepare_grasp_callback, this);
            _grasp_service = _nh.advertiseService("grasp",
                &VersaballNode::_grasp_callback, this);
            _release_service = _nh.advertiseService("release",
                &VersaballNode::_release_callback, this);
            _state_service = _nh.advertiseService("get_state",
                &VersaballNode::_state_callback, this);

            if (_prepare_grasp_service && _grasp_service && _release_service &&
                _state_service)
            {
                success = true;
            }
        }
        else
        {
            ROS_ERROR_STREAM("The service \""<<phidgets_service<<"\" was not"
                "found");
        }

        return success;
    }

    versaball_state VersaballNode::state()
    {
        return _current_state;
    }

    const std::string VersaballNode::state_str()
    {
        std::string state;
        switch(_current_state)
        {
        case neutral:
            state = "neutral";
            break;
        case soft:
            state = "soft";
            break;
        case jammed:
            state = "jammed";
            break;
        }

        return state;
    }

    void VersaballNode::_hardware_setup()
    {
        // Sub-actions for the grasp preparation
        _prepare_grasp_a.push_back(action_t(ros::Duration(0), true,  _pressure_pump ));
        _prepare_grasp_a.push_back(action_t(ros::Duration(0), true,  _pressure_valve));
        _prepare_grasp_a.push_back(action_t(ros::Duration(0), false, _pressure_pump ));
        _prepare_grasp_a.push_back(action_t(ros::Duration(0), false, _pressure_valve));

        // Sub-actions for grasping
        _grasp_a.push_back(action_t(ros::Duration(0), true,  _void_pump ));
        _grasp_a.push_back(action_t(ros::Duration(0), true,  _void_valve));
        _grasp_a.push_back(action_t(ros::Duration(0), false, _void_pump ));
        _grasp_a.push_back(action_t(ros::Duration(0), false, _void_valve));

        // Sub-actions for release (when in jammed state)
        // put some air in the Versaball
        _release_grasp_a.push_back(action_t(ros::Duration(0), true,  _pressure_pump));
        _release_grasp_a.push_back(action_t(ros::Duration(0), true,  _pressure_valve));
        // equalise Versaball's pressure with atmospheric pressure
        _release_grasp_a.push_back(action_t(ros::Duration(0), true,  _void_valve));
        // stop it all
        _release_grasp_a.push_back(action_t(ros::Duration(0), false, _void_valve));
        _release_grasp_a.push_back(action_t(ros::Duration(0), false, _pressure_pump));
        _release_grasp_a.push_back(action_t(ros::Duration(0), false, _pressure_valve));

        // Sub-actions for release (when in soft state)
        _release_prepare_grasp_a.push_back(action_t(ros::Duration(0), true,  _pressure_valve));
        _release_prepare_grasp_a.push_back(action_t(ros::Duration(0), true,  _void_valve));
        _release_prepare_grasp_a.push_back(action_t(ros::Duration(0), false, _void_valve));
        _release_prepare_grasp_a.push_back(action_t(ros::Duration(0), false, _pressure_valve));
    }

    void VersaballNode::_do_transition(std::list<action_t> actions_list)
    {
        // time at which we start the grasping sequence
        ros::Time start_time = ros::Time::now();
        // The loop bellow will run each millisecond (1000Hz), in theory
        ros::Rate rate(1000);

        // Regularly check for each remaining action whether it's time to run it
        while (!actions_list.empty())
        {
            ros::Duration relative_time = ros::Time::now() - start_time;

            // Scan actions not yet executed
            std::list<action_t>::iterator action;
            action=actions_list.begin();
            while (action!=actions_list.end())
            {
                // If it's time to execute it, do so and erase it from the list
                if (_execute_timed_action(*action, relative_time))
                    actions_list.erase(action++);
                else
                    ++action;
            }

            rate.sleep();
        }
    }

    bool VersaballNode::_execute_timed_action(const action_t& action,
        const ros::Duration& relative_time)
    {
        std::string description = std::string(action.output_state?"enable":"disable")
            + " " + action.effector.name;

        if (action.trigger <= relative_time)
        {
            if (_set_phidgets_state(action.effector.index, action.output_state))
                ROS_DEBUG_STREAM("successive service call to " << description);
            else
                ROS_DEBUG_STREAM("failed service call to " << description);

            return true;
        }
        return false;
    }

    bool VersaballNode::_set_phidgets_state(uint8_t index, uint16_t state)
    {
        phidgets_interface_kit::setState service_call;
        service_call.request.index = index;
        service_call.request.state = state;

        // querry a service to set the state of one of the relays controlling
        // our valves and pumps
        ros::ServiceClient client =
            _nh.serviceClient<phidgets_interface_kit::setState>("/phidgets_ik/set_state");

        return client.call(service_call);
    }

    bool VersaballNode::_prepare_grasp_callback(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res)
    {
        if (neutral == _current_state)
        {
            // put a little pressure in, to soften the Versaball
            _do_transition(_prepare_grasp_a);
            _current_state = soft;
            res.success = true;
        }
        else
        {
            res.success = false;
            res.message = "Cannot prepare for grasping when in state " +
                state_str();
        }
        return true;
    }

    bool VersaballNode::_grasp_callback(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res)
    {
        if (soft == _current_state)
        {
            // grasp
            _do_transition(_grasp_a);
            // enable periodic air pumping
            _keep_grasp_timer = _nh.createTimer(_keep_grasp_period,
                &VersaballNode::_keep_grasp_callback, this);
            _current_state = jammed;
            res.success = true;
        }
        else
        {
            res.success = false;
            res.message = "Cannot grasp when in state " + state_str();
        }
        return true;
    }

    bool VersaballNode::_release_callback(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res)
    {
        if (soft == _current_state)
        {
            // neutralise pressure
            _do_transition(_release_prepare_grasp_a);
            _current_state = neutral;
            res.success = true;
        }
        else if (jammed == _current_state)
        {
            // disable periodic air pumping
            _keep_grasp_timer.stop();
            // push som air, then neutralise pressure
            _do_transition(_release_grasp_a);
            _current_state = neutral;
            res.success = true;
        }
        else
        {
            res.success = false;
            res.message = "Cannot release when in state " + state_str();
        }
        return true;
    }

    bool VersaballNode::_state_callback(GetState::Request &req,
        GetState::Response &res)
    {
        res.state = state_str();
        return true;
    }

    void VersaballNode::_keep_grasp_callback(const ros::TimerEvent& event)
    {
        _set_phidgets_state(_void_valve.index, true);
        _set_phidgets_state(_void_pump.index, true);
        _keep_grasp_duration.sleep();
        _set_phidgets_state(_void_pump.index, false);
        _set_phidgets_state(_void_valve.index, false);
    }

    void VersaballNode::dynamic_reconfigure_cb(versaball::versaballConfig &config, uint32_t level)
    {
        // For the grasping preparation
        std::list<action_t>::iterator pg_a = _prepare_grasp_a.begin();
        pg_a->trigger = ros::Duration(config.pg_start_pressure*1e-3);        pg_a++;
        pg_a->trigger = ros::Duration(config.pg_open_presure_valve*1e-3);    pg_a++;
        pg_a->trigger = ros::Duration(config.pg_stop_pressure*1e-3);         pg_a++;
        pg_a->trigger = ros::Duration(config.pg_close_pressure_valve*1e-3);

        // For the grasping
        std::list<action_t>::iterator g_a = _grasp_a.begin();
        g_a->trigger = ros::Duration(config.g_start_void*1e-3);          g_a++;
        g_a->trigger = ros::Duration(config.g_open_void_valve*1e-3);     g_a++;
        g_a->trigger = ros::Duration(config.g_stop_void*1e-3);           g_a++;
        g_a->trigger = ros::Duration(config.g_close_void_valve*1e-3);

        // For the release when in jammed state (after a "grasp")
        std::list<action_t>::iterator rg_a = _release_grasp_a.begin();
        rg_a->trigger = ros::Duration(config.rg_start_pressure*1e-3);        rg_a++;
        rg_a->trigger = ros::Duration(config.rg_open_presure_valve*1e-3);    rg_a++;
        rg_a->trigger = ros::Duration(config.rg_open_void_valve*1e-3);       rg_a++;
        rg_a->trigger = ros::Duration(config.rg_close_void_valve*1e-3);      rg_a++;
        rg_a->trigger = ros::Duration(config.rg_stop_pressure*1e-3);         rg_a++;
        rg_a->trigger = ros::Duration(config.rg_close_pressure_valve*1e-3);

        // For the release when in soft state (after a "prepare grasp")
        std::list<action_t>::iterator rpg_a = _release_prepare_grasp_a.begin();
        rpg_a->trigger = ros::Duration(config.rp_open_presure_valve*1e-3);   rpg_a++;
        rpg_a->trigger = ros::Duration(config.rp_open_void_valve*1e-3);      rpg_a++;
        rpg_a->trigger = ros::Duration(config.rp_close_void_valve*1e-3);     rpg_a++;
        rpg_a->trigger = ros::Duration(config.rp_close_pressure_valve*1e-3);

        // For the background task maintaining the void for grasping
        _keep_grasp_period  = ros::Duration(config.keep_period*1e-3);
        _keep_grasp_duration = ros::Duration(config.keep_duration*1e-3);
    }
}