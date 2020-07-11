/** \brief
 *  \file phidgets_interface_kit_888.cpp
 *  \author Peter Lepej
 *  \date 21.9.2015
 *  \version 1.0
 *  \place: FKBV, Pivola 10, Hoče, Slovenia
 */

#ifndef phidgets_spatial_hpp___
#define phidgets_spatial_hpp___ //header is included only once

//basic
#include <ros/ros.h>
#include <ros/service_server.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <sstream>

#include <libphidgets/phidget21.h>
//#include <phidgets_api/phidget.h>

#include <tf/tf.h>
#include <tf2/transform_datatypes.h>

//messagess
#include "rovitis_phidgets/phidget_gps.h"

float gps_latitude = 0;
float gps_longitude = 0;
float gps_altitude = 0;
float gps_heading = 0;
float gps_velocity = 0;
int gps_status = 0;
std::string gps_datetime;

ros::Time time_now;
ros::Time time_zero_;
std::string sp_frame_;

bool initialised_= false;

using namespace std;

namespace phidgets_gps  //split up code more classes whit the same name
{
    class phidgets_gps
    {
        private:

        /** \brief Copyconstruction is not allowed.
         */
        phidgets_gps(const phidgets_gps &src);

        protected:
        ros::NodeHandle nh_;

        //topics handlers
        ros::Publisher pub_gps_;

        public:

        //global device handler
        CPhidgetSpatialHandle gps;

        //global arguments
        int serial_number_;
        int curr_serial_number_;
        string device_label_;
        string topic_name_;

        int wait_for_dev_delay_;
        int publish_frequency_;
        bool reset_;

        /** \brief Standard construktor.
         *
         */
        phidgets_gps(ros::NodeHandle nh);

        /** \brief Destructor.
         *
         */
        virtual ~phidgets_gps(); //always virtual

        //initialization function
        void init();
        //connect device
        bool connect_dev();
        //display device info
        void display_generic_properties(CPhidgetHandle phid);
        //publish data
        void publish_data();
        //velocity callback
        //void velocityCommandCallback(const geometry_msgs::Twist::ConstPtr& ptr);
        //disconnecte phidgets
        void disconnect();

    };
}

#endif
