/** \brief
 *  \file phidgets_interface_kit_888.hpp
 *  \author Peter Lepej
 *  \date 21.9.2015
 *  \version 1.0
 *  \place: FKBV, Pivola 10, Hoče, Slovenia
 */

#include "rovitis_phidgets/phidgets_stepper_bipolar.hpp"

int mot_index;
double mot_pose;

namespace phidgets_stepper_bipolar
{
    /****************************************************************
     * Here we can set up the parameters
     */
    phidgets_stepper_bipolar::phidgets_stepper_bipolar(ros::NodeHandle nh)    //constructor for the class
    : nh_(nh),
    phandle(0)
    {
        ros::NodeHandle private_nh("~");

        private_nh.param<int>("dev_serial_number", serial_number_, 322218); //130505=Axis 322218=dosing
        private_nh.param<std::string>("dev_serial_label", device_label_, string("phidgets_stepper_bipolar"));
        private_nh.param<std::string>("frame", pfc_frame_, string("/phidget_sb"));
        private_nh.param<int>("wait_for_dev_delay", wait_for_dev_delay_, 10000);
        private_nh.param<int>("publish_frequency", publish_frequency_, 5);
        private_nh.param<std::string>("topic_name", topic_name_, string("/phidgets_stepper_bipolar_pose"));
        private_nh.param<std::string>("set_pose_service_name", service_name_, string("phidgets_stepper_bipolar_set"));
        private_nh.param<std::string>("vel_cmd_topic_name", vel_cmd_topic_name_, string("/cmd_vel_reg"));

        private_nh.param<bool>("motors_active", motors_active_, true);
        private_nh.param<double>("max_angular_velocity", max_angular_velocity_, 0.5);
        private_nh.param<double>("max_linear_velocity", max_linear_velocity_, 0.5);
        private_nh.param<int>("motor_contorller_id", motor_contorller_id_, 0);
        private_nh.param<double>("motor_acceleration", motor_acceleration_, 4000);
        private_nh.param<double>("motor_velocity", motor_velocity_, 32000);
        private_nh.param<double>("robot_dist_from_center_to_wheel", robot_dist_from_center_to_wheel_, 1.0);

    }
    /****************************************************************
     *
     */
    phidgets_stepper_bipolar::~phidgets_stepper_bipolar()
    {
    }
    /*****************************************************************************************************************
     * Initialization
     */
    void phidgets_stepper_bipolar::init()
    {
        //connect device
        ROS_INFO("Connecting device ... stepper_bipolar");
        if (connect_dev())
        {
            ROS_INFO("Connection to stepper_bipolar successfull!");
            initialised_ = true;
        }
        else
        {
            ROS_ERROR("There is no device stepper_bipolar connected! Node shutdown!");
            initialised_ = false;

            CPhidget_close((CPhidgetHandle)phandle);
            CPhidget_delete((CPhidgetHandle)phandle);

            ros::shutdown();
        }

        if(initialised_)
        {
            if(serial_number_ == curr_serial_number_)
            {
                //set device label
                const char* cname = device_label_.c_str();
                CPhidget_setDeviceLabel((CPhidgetHandle)phandle, cname);

                //create topics to publish data
                pub_ = nh_.advertise<rovitis_phidgets::phidget_stepper_bipolar>(topic_name_, 1);

                //subscribe to ifk2_topic
                ik_sub_ = nh_.subscribe("/phidgets_interface_kit_888", 1, &phidgets_stepper_bipolar::ifkCallback, this);

                //create service
                service_ = nh_.advertiseService(service_name_, &phidgets_stepper_bipolar::set_pose, this);

                //subscribe to cmd_vel
                cmd_vel_sub_ = nh_.subscribe(vel_cmd_topic_name_, 1, &phidgets_stepper_bipolar::velocityCommandCallback, this);


            }else
            {
                ROS_ERROR("Defined device serial number: %i dose not match to current connected device serial number: %i!", serial_number_,curr_serial_number_);
            }
        }
    }

    int CCONV AttachHandler(CPhidgetHandle phid, void *userptr)
    {
        CPhidgetFrequencyCounterHandle pfc_handle = (CPhidgetFrequencyCounterHandle)phid;

        ROS_INFO("Run attached stepper_bipolar handler!");
        return 0;
    }

    int CCONV DetachHandler(CPhidgetHandle phid, void *userptr)
    {
        ROS_INFO("Detach stepper_bipolar handler!");
        return 0;
    }

    int CCONV ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *errorStr)
    {
        ROS_ERROR("Error event stepper_bipolar: %s\n",errorStr);
        return 0;
    }

    void phidgets_stepper_bipolar::display_generic_properties(CPhidgetHandle phid)
    {
        int sernum, version, num_inputs, num_motors;
        const char *deviceptr;
        CPhidget_getDeviceType(phid, &deviceptr);
        CPhidget_getSerialNumber(phid, &sernum);
        CPhidget_getDeviceVersion(phid, &version);
        curr_serial_number_ = sernum;
        ROS_INFO("Device Name: %s Version: %d SerialNumber: %d",deviceptr, version, sernum);

        CPhidgetStepper_getMotorCount(phandle, &num_motors);
        //ROS_INFO("Number of motors %d", num_motors);

        return;
    }

    int PositionChangeHandler(CPhidgetStepperHandle stepper, void *usrptr, int Index, long long Value)
    {

        mot_index = Index;
        mot_pose = Value;
        ROS_INFO("Motor %d Current position %lld", Index, Value);
        return 0;
    }

    bool phidgets_stepper_bipolar::set_pose(rovitis_phidgets::phidget_stepper_bipolar_set::Request &req, rovitis_phidgets::phidget_stepper_bipolar_set::Response &res)
    {
        ROS_INFO("Request received: Mot index: %i Reference Acceleration: %f, Reference Velocity: %f, Target position: %f Motor on: %i Reset pose: %i", req.motor_index, req.acceleration, req.velocity, req.position, req.engage, req.reset_pose);

        int motor_index = (int)req.motor_index;

        //turn motor on
        //if (!req.engage)
        {
            CPhidgetStepper_setEngaged(phandle, motor_index, 0);
            usleep(500);
        }

        //set motor parameters
        CPhidgetStepper_setAcceleration (phandle, motor_index, (double)req.acceleration);
        CPhidgetStepper_setVelocityLimit (phandle, motor_index, (double)req.velocity);

        CPhidgetStepper_setCurrentLimit(phandle, motor_index, 1.0);

        CPhidgetStepper_setCurrentPosition(phandle, motor_index, 0);
        CPhidgetStepper_setTargetPosition(phandle, motor_index, 0);
        usleep(500);

        if (req.engage)
        {
            CPhidgetStepper_setEngaged(phandle, motor_index, 1);
            usleep(500);
        }

        CPhidgetStepper_setTargetPosition (phandle, motor_index, req.position);

        //reset motor pose
        if (req.reset_pose)
        {
            //CPhidgetStepper_setCurrentPosition(phandle, 0, 0);
        }


        return true;
    }

    bool phidgets_stepper_bipolar::connect_dev()
    {

        const char *err;
        int result;

        CPhidgetStepper_create(&phandle);

        CPhidget_set_OnAttach_Handler((CPhidgetHandle)phandle, AttachHandler, NULL);
        CPhidget_set_OnDetach_Handler((CPhidgetHandle)phandle, DetachHandler, NULL);
        CPhidget_set_OnError_Handler((CPhidgetHandle)phandle, ErrorHandler, NULL);

        CPhidgetStepper_set_OnPositionChange_Handler(phandle, PositionChangeHandler, NULL);

        CPhidget_open((CPhidgetHandle)phandle, serial_number_);

        //Wait for 10 seconds, otherwise exit
        if(result = CPhidget_waitForAttachment((CPhidgetHandle)phandle, wait_for_dev_delay_))
        {

            CPhidget_getErrorDescription(result, &err);
            printf("Problem waiting for attachment: %s\n", err);
            return false;
        }

        display_generic_properties((CPhidgetHandle)phandle);
        return true;
    }

    void phidgets_stepper_bipolar::publish_data()
    {

        //publish data
        rovitis_phidgets::phidget_stepper_bipolar data;

        data.header.frame_id = pfc_frame_;
        data.index = mot_index;
        data.pose = mot_pose;

        pub_.publish(data);

    }

    void phidgets_stepper_bipolar::disconnect()
    {
        ROS_INFO("Closing...");
        CPhidget_close((CPhidgetHandle)phandle);
        CPhidget_delete((CPhidgetHandle)phandle);
    }

    void phidgets_stepper_bipolar::ifkCallback(const rovitis_phidgets::phidget_interface_kit_888 data)
    {
       contelec_X28_ = data.an_ch4;
       //ROS_INFO("IFK callback received %f pose", Contelec X28 );

    }


    void phidgets_stepper_bipolar::velocityCommandCallback(const geometry_msgs::Twist::ConstPtr& ptr)
    {
        ROS_INFO_ONCE("Velocity Command Received! Cmd Vel is published at same frequency as input /cmd_vel topic!");

        geometry_msgs::Twist cmd = *ptr;
        current_linear_velocity_ = cmd.linear.x;
        current_angular_velocity_ = cmd.angular.z;

        if ((current_angular_velocity_!=0) || (current_angular_velocity_!=0))
        {
            ROS_INFO("cmd_vel %.3f angular %.3f", current_linear_velocity_, current_angular_velocity_);
        }


        //limit velocities
        if(fabs(current_angular_velocity_) > fabs(max_angular_velocity_))
        {
            current_angular_velocity_ = (fabs(current_angular_velocity_)/current_angular_velocity_)*max_angular_velocity_;
        }
        if(fabs(current_linear_velocity_) > fabs(max_linear_velocity_))
        {
            current_linear_velocity_ = (fabs(current_linear_velocity_)/current_linear_velocity_)*max_linear_velocity_;
        }

        if(motors_active_ == true)
        {
            //turn motor off
//            if (!req.engage)
//            {
//                CPhidgetStepper_setEngaged(phandle, motor_index, 0);
//            }

//            //reset motor pose
//            if (req.reset_pose)
//            {
//                CPhidgetStepper_setCurrentPosition(phandle, 0, 0);
//            }

            //calculate transformations
            double motor_vel;
            if(motor_contorller_id_ == 0)
            double motor_vel= current_linear_velocity_ - current_angular_velocity_*robot_dist_from_center_to_wheel_;
            if(motor_contorller_id_ == 1)
            double motor_vel = current_linear_velocity_ + current_angular_velocity_*robot_dist_from_center_to_wheel_;

            motor_set_pose_ = motor_vel * 1000; //??????????????? make transforms!!!!

            //set motor parameters
            CPhidgetStepper_setAcceleration (phandle, motor_contorller_id_, motor_acceleration_);
            CPhidgetStepper_setVelocityLimit (phandle, motor_contorller_id_, motor_velocity_);
            CPhidgetStepper_setTargetPosition (phandle, motor_contorller_id_, motor_set_pose_);

//            if (req.engage)
//            {
//                CPhidgetStepper_setEngaged(phandle, motor_contorller_id_, 1);
//            }

        }
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "phidgets_frequency_counter");
    ros::NodeHandle nh;

    //For node registering
    phidgets_stepper_bipolar::phidgets_stepper_bipolar phidgets_stepper_bipolar_handler(nh);

    //initialization
    phidgets_stepper_bipolar_handler.init();

    ros::Rate loop_rate(phidgets_stepper_bipolar_handler.publish_frequency_);  //daj to v parametre
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();

        phidgets_stepper_bipolar_handler.publish_data();
    }

    phidgets_stepper_bipolar_handler.disconnect();
    return 0;
}
