/** \brief
 *  \file phidgets_interface_kit_888.cpp
 *  \author Peter Lepej
 *  \date 21.9.2015
 *  \version 1.0
 *  \place: FKBV, Pivola 10, Hoče, Slovenia
 */

#ifndef phidgets_motor_control_hc_hpp___
#define phidgets_motor_control_hc_hpp___ //header is included only once

//basic
#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <sstream>

//phidgets stuff
#include <libphidgets/phidget21.h>
//#include <phidgets_api/phidget.h>

//messagess
#include <geometry_msgs/Twist.h>

#include "rovitis_phidgets/phidget_motor_control_hc.h"

//ifkit2
#include "rovitis_phidgets/phidget_interface_kit_888_set.h"
#include "rovitis_phidgets/phidget_interface_kit_888.h"

//services
//#include "rovitis_row_navigation/rovitis_start_row_navigation.h"

//subscribe
//#include "rovitis_remote_control/rovitis_remote_control_info.h"


using namespace std;

namespace phidgets_motor_control_hc  //split up code more classes whit the same name
{
    class phidgets_motor_control_hc
    {
        private:

        /** \brief Copyconstruction is not allowed.
         */
        phidgets_motor_control_hc(const phidgets_motor_control_hc &src);


        protected:
        ros::NodeHandle nh_;

        //service handlers
        ros::ServiceClient start_navi_service_;

        //phidgeds boards clients
        ros::ServiceClient pik_888_1_client_;

        //topics handlers
        ros::Publisher pub_;
        ros::Subscriber cmd_vel_sub_;
        ros::Subscriber pik_888_1_sub_;
	ros::Subscriber manual_controller_sub_;

        public:

        //global device handler
        CPhidgetMotorControlHandle phandle;

        //global arguments
        int serial_number_;
        int curr_serial_number_;
        bool initialised_;
	bool rovitis_auto_;
        string device_label_;
        string topic_name_;
        string pfc_frame_;
        string vel_cmd_topic_name_;
        int wait_for_dev_delay_;
        int publish_frequency_;
        bool on_exec_;
        bool motors_active_;
        double acceleration_;
        double current_throttle_pose_;
        double max_throtle_val_;
        double min_throtle_val_;
        double throttle_position_;
	double curr_thr_acc_;

        int time_ms_to_stop;

	double throttle_calc_position_;
	double old_throttle_position_;
	double old_throttle_pose_;

        /** \brief Standard construktor.
         *
         */
        phidgets_motor_control_hc(ros::NodeHandle nh);

        /** \brief Destructor.
         *
         */
        virtual ~phidgets_motor_control_hc(); //always virtual

        //initialization function
        void init();
        //connect device
        bool connect_dev();
        //display device info
        void display_generic_properties(CPhidgetHandle phid);
        //publish data
        void publish_data();
        //velocity callback
        void throttleCommandCallback(const geometry_msgs::Twist::ConstPtr& ptr);
        //stop motors
        void stop_motors();
        //disconnecte phidgets
        void disconnect();
        //enable motors
        void enableMotorControl();
        void disableMotorControl();
        void ifk2Callback(const rovitis_phidgets::phidget_interface_kit_888 data);
	      //void manualCallback(const rovitis_remote_control::teleop_rovitis_info data);


    };
}

#endif
