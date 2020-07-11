/** \brief
 *  \file phidgets_interface_kit_888.cpp
 *  \author Peter Lepej
 *  \date 21.9.2015
 *  \version 1.0
 *  \place: FKBV, Pivola 10, Hoče, Slovenia
 */

#ifndef phidgets_frequency_counter_hpp___
#define phidgets_frequency_counter_hpp___ //header is included only once

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
#include "rovitis_phidgets/phidget_frequency_counter.h"
//services
#include "rovitis_phidgets/phidget_frequency_counter_reset.h"

using namespace std;

namespace phidgets_frequency_counter  //split up code more classes whit the same name
{
    class phidgets_frequency_counter
    {
        private:

        /** \brief Copyconstruction is not allowed.
         */
        phidgets_frequency_counter(const phidgets_frequency_counter &src);


        protected:
        ros::NodeHandle nh_;

        //service handlers
        ros::ServiceServer reset_service_;

        //topics handlers
        ros::Publisher pcf_pub_;

        public:

        //global device handler
        CPhidgetFrequencyCounterHandle pfc_handle;

        //global arguments
        int serial_number_;
        int curr_serial_number_;
        bool initialised_;
        string device_label_;
        string pfc_topic_name_;
        string pfc_reset_service_name_;
        string pfc_frame_;
        int wait_for_dev_delay_;
        int publish_frequency_;
        bool enable_ch0_;
        bool enable_ch1_;
        string filter_ch0_;
        string filter_ch1_;
        bool reset_;

        /** \brief Standard construktor.
         *
         */
        phidgets_frequency_counter(ros::NodeHandle nh);

        /** \brief Destructor.
         *
         */
        virtual ~phidgets_frequency_counter(); //always virtual

        //initialization function
        void init();
        //connect device
        bool connect_dev();
        //display device info
        void display_generic_properties(CPhidgetHandle phid);
        //service callback
        bool pfc_reset(rovitis_phidgets::phidget_frequency_counter_reset::Request  &req, rovitis_phidgets::phidget_frequency_counter_reset::Response &res);
        //publish data
        void publish_data();
        //disconnecte phidgets
        void disconnect();
    };
}

#endif
