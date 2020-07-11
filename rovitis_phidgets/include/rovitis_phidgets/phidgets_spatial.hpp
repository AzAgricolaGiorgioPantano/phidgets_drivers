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

#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf2/transform_datatypes.h>

//messagess
#include "rovitis_phidgets/phidget_spatial.h"
#include "rovitis_phidgets/phidget_spatial_set.h"

const float G = 1; // original value 9.81;

ros::Time time_now;
ros::Time time_zero_;
bool initialized_;
ros::Time last_imu_time_;
std::string sp_frame_;

ros::Time imu_header_time;

// define linear acceleration
double imu_linear_acceleration_x = 0;
double imu_linear_acceleration_y = 0;
double imu_linear_acceleration_z = 0;

// define angular velocities
double imu_angular_velocity_x = 0;
double imu_angular_velocity_y = 0;
double imu_angular_velocity_z = 0;
double imu_angular_velocity_z_old = 0;

// define magnetic field
double mag_vector_x;
double mag_vector_y;
double mag_vector_z;

bool initialised_= false;

using namespace std;

namespace phidgets_spatial  //split up code more classes whit the same name
{
    class phidgets_spatial
    {
        private:

        /** \brief Copyconstruction is not allowed.
         */
        phidgets_spatial(const phidgets_spatial &src);

        protected:
        ros::NodeHandle nh_;

        //topics handlers
        ros::Publisher  pub_;

        ros::Publisher  cal_publisher_;

        ros::Publisher pub_imu_;

        //service handlers
        ros::ServiceServer cal_srv_;
        ros::ServiceServer reset_service_;

        //service handlers
        ros::ServiceServer service_;

        //cmd_vel
        ros::Subscriber cmd_vel_sub_;

        public:

        //global device handler
        CPhidgetSpatialHandle spatial;

        bool calibrateService(std_srvs::Empty::Request  &req,
                              std_srvs::Empty::Response &res);

        //global arguments
        int serial_number_;
        int curr_serial_number_;
        string device_label_;
        string topic_name_;

        string service_name_;
        int wait_for_dev_delay_;
        int publish_frequency_;
        bool reset_;

        /** \brief Standard construktor.
         *
         */
        phidgets_spatial(ros::NodeHandle nh);

        /** \brief Destructor.
         *
         */
        virtual ~phidgets_spatial(); //always virtual

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
        //service callback
        bool set_pose(rovitis_phidgets::phidget_spatial_set::Request &req, rovitis_phidgets::phidget_spatial_set::Response &res);

        void calibrate();
        void dataHandler(CPhidgetSpatial_SpatialEventDataHandle* data, int count);



    };
}

#endif
