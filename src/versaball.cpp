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
        _void_valve("void valve", 3)
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
        std::string phidgets_service = "phidgets_ik/set_states";

        ROS_INFO_STREAM("Waiting for \""<<phidgets_service<<"\" service"
            "(2 seconds each).");
        if (ros::service::waitForService(phidgets_service, 2000))
        {

            _grasp_service = _nh.advertiseService("grasp",
                &VersaballNode::grasp_callback, this);
            _release_service = _nh.advertiseService("release",
                &VersaballNode::release_callback, this);

            if (_grasp_service && _release_service)
                success = true;
        }
        else
        {
            ROS_ERROR_STREAM("The service \""<<phidgets_service<<"\" was not"
                "found");
        }

        return success;
    }

    bool VersaballNode::grasp_callback(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res)
    {
        // time at which we start the grasping sequence
        ros::Time start_time = ros::Time::now();
        // The loop bellow will run each millisecond (1000Hz), in theory
        ros::Rate rate(1000);

        std::list<action_t> actions_list = _prepare_grasp_a;

        while (!actions_list.empty())
        {
            ros::Duration now = ros::Time::now() - start_time;

            std::list<action_t>::iterator action;
            action=actions_list.begin();
            while (action!=actions_list.end())
            {
                if (_timed_action(*action, now))
                    actions_list.erase(action++);
                else
                    ++action;
            }

            rate.sleep();
        }

        return true;
    }

    bool VersaballNode::release_callback(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res)
    {
        return true;

    }

    void VersaballNode::dynamic_reconfigure_cb(versaball::versaballConfig &config, uint32_t level)
    {
        std::list<action_t>::iterator pg_a = _prepare_grasp_a.begin();
        pg_a->offset = ros::Duration(config.pg_start_pressure*1e-3);       pg_a++;
        pg_a->offset = ros::Duration(config.pg_open_presure_valve*1e-3);   pg_a++;
        pg_a->offset = ros::Duration(config.pg_close_pressure_valve*1e-3); pg_a++;
        pg_a->offset = ros::Duration(config.pg_stop_pressure*1e-3);

        std::list<action_t>::iterator g_a = _grasp_a.begin();
        g_a->offset = ros::Duration(config.g_start_void*1e-3);         g_a++;
        g_a->offset = ros::Duration(config.g_open_void_valve*1e-3);    g_a++;
        g_a->offset = ros::Duration(config.g_close_void_valve*1e-3);   g_a++;
        g_a->offset = ros::Duration(config.g_stop_void*1e-3);

        std::list<action_t>::iterator rg_a = _release_grasp_a.begin();
        rg_a->offset = ros::Duration(config.rg_start_pressure*1e-3);        rg_a++;
        rg_a->offset = ros::Duration(config.rg_open_presure_valve*1e-3);    rg_a++;
        rg_a->offset = ros::Duration(config.rg_start_void*1e-3);            rg_a++;
        rg_a->offset = ros::Duration(config.rg_open_void_valve*1e-3);       rg_a++;
        rg_a->offset = ros::Duration(config.rg_close_void_valve*1e-3);      rg_a++;
        rg_a->offset = ros::Duration(config.rg_stop_void*1e-3);             rg_a++;
        rg_a->offset = ros::Duration(config.rg_close_pressure_valve*1e-3);  rg_a++;
        rg_a->offset = ros::Duration(config.rg_stop_pressure*1e-3);

        std::list<action_t>::iterator rpg_a = _release_prepare_grasp_a.begin();
        rpg_a->offset = ros::Duration(config.rp_start_pressure*1e-3);       rpg_a++;
        rpg_a->offset = ros::Duration(config.rp_open_presure_valve*1e-3);   rpg_a++;
        rpg_a->offset = ros::Duration(config.rp_start_void*1e-3);           rpg_a++;
        rpg_a->offset = ros::Duration(config.rp_open_void_valve*1e-3);      rpg_a++;
        rpg_a->offset = ros::Duration(config.rp_close_void_valve*1e-3);     rpg_a++;
        rpg_a->offset = ros::Duration(config.rp_stop_void*1e-3);            rpg_a++;
        rpg_a->offset = ros::Duration(config.rp_close_pressure_valve*1e-3); rpg_a++;
        rpg_a->offset = ros::Duration(config.rp_stop_pressure*1e-3);
    }

    versaball_state VersaballNode::state()
    {
        return _current_state;
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
        // enable the void system, to equalise Versaball's pressure with
        // atmospheric pressure
        _release_grasp_a.push_back(action_t(ros::Duration(0), true,  _void_pump));
        _release_grasp_a.push_back(action_t(ros::Duration(0), true,  _void_valve));
        // stop it all
        _release_grasp_a.push_back(action_t(ros::Duration(0), false, _void_pump));
        _release_grasp_a.push_back(action_t(ros::Duration(0), false, _void_valve));
        _release_grasp_a.push_back(action_t(ros::Duration(0), false, _pressure_pump));
        _release_grasp_a.push_back(action_t(ros::Duration(0), false, _pressure_valve));

        // Sub-actions for release (when in soft state)
        _release_prepare_grasp_a.push_back(action_t(ros::Duration(0), true,  _pressure_pump));
        _release_prepare_grasp_a.push_back(action_t(ros::Duration(0), true,  _pressure_valve));
        _release_prepare_grasp_a.push_back(action_t(ros::Duration(0), true,  _void_pump));
        _release_prepare_grasp_a.push_back(action_t(ros::Duration(0), true,  _void_valve));
        _release_prepare_grasp_a.push_back(action_t(ros::Duration(0), false, _void_pump));
        _release_prepare_grasp_a.push_back(action_t(ros::Duration(0), false, _void_valve));
        _release_prepare_grasp_a.push_back(action_t(ros::Duration(0), false, _pressure_pump));
        _release_prepare_grasp_a.push_back(action_t(ros::Duration(0), false, _pressure_valve));
    }

    bool VersaballNode::prepare_grasp()
    {
        if (neutral == _current_state)
        {
            // prepare grasp
            _current_state = soft;
            return true;
        }
        else
            return false;
    }

    bool VersaballNode::grasp()
    {
        if (soft == _current_state)
        {
            // grasp
            // enable periodic air pumping
            _current_state = jammed;
            return true;
        }
        else
            return false;
    }

    bool VersaballNode::release()
    {
        if (soft == _current_state)
        {
            // neutralise pressure
            return true;
        }
        else if (jammed == _current_state)
        {
            // disable periodic air pumping
            // go positive pressure, then neutralise pressure
            return true;
        }
        else
            return false;
    }

    bool VersaballNode::_timed_action(const action_t& action, const ros::Duration& now)
    {
        std::string description = std::string(action.output_state?"enable":"disable") + " "
            + action.effector.name;

        if (action.offset <= now)
        {
            if (_set_phidgets_state(action.effector.index, action.output_state))
                ROS_DEBUG_STREAM("successfully " << description);
            else
                ROS_DEBUG_STREAM("unable to " << description);

            return true;
        }
        return false;
    }

    bool VersaballNode::_set_phidgets_state(uint8_t index, uint16_t state)
    {
        phidgets_interface_kit::setState service_call;
        service_call.request.index = index;
        service_call.request.state = state;

        ros::ServiceClient client =
            _nh.serviceClient<phidgets_interface_kit::setState>("/phidgets_ik/set_state");

        return client.call(service_call);
    }
}