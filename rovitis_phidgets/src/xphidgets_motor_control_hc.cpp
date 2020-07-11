/** \brief
 *  \file phidgets_interface_kit_888.hpp
 *  \author Peter Lepej
 *  \date 21.9.2015
 *  \version 1.0
 *  \place: FKBV, Pivola 10, Hoče, Slovenia
 */

#include "rovitis_phidgets/phidgets_motor_control_hc.hpp"

double in_change_ch0;
double in_change_ch1;
double curr_change_ch0;
double curr_change_ch1;
double vel_change_ch0;
double vel_change_ch1;
double getAcceleration_ch0;
double getAcceleration_ch1;
double getVelocity_ch0;
double getVelocity_ch1;

namespace phidgets_motor_control_hc
{
    /****************************************************************
     * Here we can set up the parameters
     */
    phidgets_motor_control_hc::phidgets_motor_control_hc(ros::NodeHandle nh)    //constructor for the class
    : nh_(nh),
    phandle(0)
    {
        ros::NodeHandle private_nh("~");

        private_nh.param<int>("dev_serial_number", serial_number_, 147399);
        private_nh.param<std::string>("dev_serial_label", device_label_, string("phidgets_motor_control_hc"));
        private_nh.param<std::string>("frame", pfc_frame_, string("/phidget_mc"));
        private_nh.param<int>("wait_for_dev_delay", wait_for_dev_delay_, 10000);
        private_nh.param<int>("publish_frequency", publish_frequency_, 5);
        private_nh.param<std::string>("topic_name", topic_name_, string("/phidgets_motor_control_hc_params"));
        private_nh.param<std::string>("vel_cmd_topic_name", vel_cmd_topic_name_, string("/cmd_vel"));

        private_nh.param<bool>("motors_active", motors_active_, true);
        private_nh.param<double>("max_throtle_val", max_throtle_val_, 650);
        private_nh.param<double>("min_throtle_val", min_throtle_val_, 980);
        private_nh.param<double>("acceleration", acceleration_, 65);
    }
    /****************************************************************
     *
     */
    phidgets_motor_control_hc::~phidgets_motor_control_hc()
    {
    }
    /*****************************************************************************************************************
     * Initialization
     */
    void phidgets_motor_control_hc::init()
    {
        //connect device
        ROS_INFO("Connecting device ... motor_control_hc");
        if (connect_dev())
        {
            ROS_INFO("Connection to motor_control_hc successfull!");
            initialised_ = true;
        }
        else
        {
            ROS_ERROR("There is no motor_control_hc connected! Node shutdown!");
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
                pub_ = nh_.advertise<rovitis_phidgets::phidget_motor_control_hc>(topic_name_, 1);

                //subscribe to cmd_vel
                cmd_vel_sub_ = nh_.subscribe(vel_cmd_topic_name_, 1, &phidgets_motor_control_hc::throttleCommandCallback, this);

                //subscribe to pik888 sn: 281641 ex ifkit2 //enable motor control
                pik_888_1_sub_ = nh_.subscribe("/phidgets_interface_kit_888_1", 1, &phidgets_motor_control_hc::ifk2Callback, this);

                //ifk2_client sn: 281641 ex ifkit2
                pik_888_1_client_ = nh_.serviceClient<rovitis_phidgets::phidget_interface_kit_888_set>("/phidgets_interface_kit_888_set_1");


            }else
            {
                ROS_ERROR("Defined device serial number: %i dose not match to current connected device serial number: %i!", serial_number_,curr_serial_number_);
            }
        }


    }

    int CCONV AttachHandler(CPhidgetHandle phid, void *userptr)
    {
        CPhidgetFrequencyCounterHandle pfc_handle = (CPhidgetFrequencyCounterHandle)phid;

        ROS_INFO("Run attached handler!");
        return 0;
    }

    int CCONV DetachHandler(CPhidgetHandle phid, void *userptr)
    {
        ROS_INFO("Detach handler!");
        return 0;
    }

    int CCONV ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *errorStr)
    {
        ROS_ERROR("Error event motor_control_hc: %s\n",errorStr);
        return 0;
    }

    void phidgets_motor_control_hc::display_generic_properties(CPhidgetHandle phid)
    {
        int sernum, version, num_inputs, num_motors;
        const char *deviceptr;
        CPhidget_getDeviceType(phid, &deviceptr);
        CPhidget_getSerialNumber(phid, &sernum);
        CPhidget_getDeviceVersion(phid, &version);
        curr_serial_number_ = sernum;
        ROS_INFO("Device Name: %s Version: %d SerialNumber: %d",deviceptr, version, sernum);

        CPhidgetMotorControl_getInputCount(phandle, &num_inputs);
        CPhidgetMotorControl_getMotorCount(phandle, &num_motors);

        ROS_INFO("Number of motors %d", num_motors);
        ROS_INFO("Number of inputs %d", num_inputs);
        return;
    }

    int CurrentChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
    {
        if(Index == 0)
        {
            curr_change_ch0 = (double)Value;
        }
        if(Index == 1)
        {
            curr_change_ch1 = (double)Value;
        }
        //ROS_INFO("Motor %d Current %.2f", Index, (float)Value);
        return 0;
    }

    int VelocityChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
    {
        if(Index == 0)
        {
            vel_change_ch0 = (double)Value;
        }
        if(Index == 1)
        {
            vel_change_ch1 = (double)Value;
        }
        //ROS_INFO("Motor %d Velocity %.2f", Index, (float)Value);
        return 0;
    }

    int InputChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, int State)
    {
        if(Index == 0)
        {
            in_change_ch0 = (double)State;
        }
        if(Index == 1)
        {
            in_change_ch1 = (double)State;
        }

        ROS_INFO("Motor input %d Inputs %d", Index, State);
        return 0;
    }


    int CPhidgetMotorControl_getVelocity(CPhidgetMotorControlHandle MC, int Index, double velocity)
    {
        if(Index == 0)
        {
            getVelocity_ch0 = (double)velocity;
        }
        if(Index == 1)
        {
            getVelocity_ch1 = (double)velocity;
        }

        ROS_INFO("Motor input %d velocity %.2f", Index, velocity);
        return 0;
    }

    bool phidgets_motor_control_hc::connect_dev()
    {

        const char *err;
        int result;

        CPhidgetMotorControl_create(&phandle);

        CPhidget_set_OnAttach_Handler((CPhidgetHandle)phandle, AttachHandler, NULL);
        CPhidget_set_OnDetach_Handler((CPhidgetHandle)phandle, DetachHandler, NULL);
        CPhidget_set_OnError_Handler((CPhidgetHandle)phandle, ErrorHandler, NULL);

        //register callbacks
        CPhidgetMotorControl_set_OnInputChange_Handler (phandle, InputChangeHandler, NULL);
        CPhidgetMotorControl_set_OnVelocityChange_Handler (phandle, VelocityChangeHandler, NULL);
        CPhidgetMotorControl_set_OnCurrentChange_Handler (phandle, CurrentChangeHandler, NULL);

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

    void phidgets_motor_control_hc::publish_data()
    {
        //publish data
        rovitis_phidgets::phidget_motor_control_hc data;

        data.header.frame_id = pfc_frame_;
        data.inchange_ch0 = in_change_ch0;
        data.inchange_ch1 = in_change_ch1;
        data.cur_ch0 = curr_change_ch0;
        data.cur_ch1 = curr_change_ch1;
        data.vel_ch0 = vel_change_ch0;
        data.vel_ch1 = vel_change_ch1;

        pub_.publish(data);
    }

    void phidgets_motor_control_hc::throttleCommandCallback(const geometry_msgs::Twist::ConstPtr& ptr)
    {
        ROS_INFO_ONCE("Throttle Command Received! Cmd Vel is published at same frequency as input /cmd_vel topic!");

        geometry_msgs::Twist cmd = *ptr;
        current_throttle_pose_ = cmd.linear.z;

	double throttle_diff_ =  current_throttle_pose_ - throttle_position_;

	double setVelocity_;
	on_exec_ = false;

        ROS_INFO("setVelocity_: %.2f Throttle Pos is: %.2f Throttle calc Pos: %.2f throttle_diff_ %.2f",setVelocity_, throttle_position_, current_throttle_pose_,throttle_diff_);


	if  (throttle_diff_ > 14 && current_throttle_pose_ != 0) //Accelerate
	{
	    setVelocity_ = -30;

            CPhidgetMotorControl_setAcceleration(phandle, 0, acceleration_);
            CPhidgetMotorControl_setVelocity(phandle, 0, setVelocity_ );

	    on_exec_ = true;
            usleep(500000);

	}

	if  (throttle_diff_ < -14 && current_throttle_pose_ != 0) //Decelerate
	{
	    setVelocity_ = 25;

            CPhidgetMotorControl_setAcceleration(phandle, 0, acceleration_);
            CPhidgetMotorControl_setVelocity(phandle, 0, setVelocity_ );

	    on_exec_ = true;
            usleep(500000);


	}

        ROS_INFO("setVelocity_: %.2f ",setVelocity_);

	if (on_exec_ && fabs(throttle_diff_) < 14)
	{
		ROS_INFO("STOP THROTTLE: Pos %.2f",throttle_position_);
                CPhidgetMotorControl_setAcceleration(phandle, 0, 0);
                CPhidgetMotorControl_setVelocity(phandle, 0, 0 );

	}

    }

    void phidgets_motor_control_hc::ifk2Callback(const rovitis_phidgets::phidget_interface_kit_888 data)
    {
       throttle_position_ = data.an_ch2;
       //ROS_INFO("IFK2 callback received %f pose", throttle_position_);

    }

    void phidgets_motor_control_hc::enableMotorControl()
    {
        rovitis_phidgets::phidget_interface_kit_888_set srv0;
        srv0.request.index = 6;
        srv0.request.value = 1;
        if (pik_888_1_client_.call(srv0))
        {
            ROS_INFO("Service called to turn on IFKIT 2, channel: %i, value: %i", srv0.request.index, srv0.request.value);
            ROS_INFO("Throttle linear actuator ON");
        }
        else
        {
            ROS_ERROR("Failed to call service ... enableMotorControl");
            //return 1;
        }
    }

    void phidgets_motor_control_hc::disableMotorControl()
    {
        rovitis_phidgets::phidget_interface_kit_888_set srv0;
        srv0.request.index = 6;
        srv0.request.value = 0;
        if (pik_888_1_client_.call(srv0))
        {
            ROS_INFO("Service called to turn on IFKIT 2, channel: %i, value: %i", srv0.request.index, srv0.request.value);
            ROS_INFO("Throttle linear actuator OFF");
        }
        else
        {
            ROS_ERROR("Failed to call service ... disableMotorControl");
            //return 1;
        }

    }

    void phidgets_motor_control_hc::stop_motors()
    {
        CPhidgetMotorControl_setVelocity(phandle, 0, 0);
        CPhidgetMotorControl_setVelocity(phandle, 1, 0);
    }

    void phidgets_motor_control_hc::disconnect()
    {
        //disable motor control
        disableMotorControl();

        ROS_INFO("Closing...");
        CPhidget_close((CPhidgetHandle)phandle);
        CPhidget_delete((CPhidgetHandle)phandle);
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "phidgets_motor_controller");
    ros::NodeHandle nh;

    //For node registering
    phidgets_motor_control_hc::phidgets_motor_control_hc phidgets_motor_control_hc_handler(nh);

    //initialization
    phidgets_motor_control_hc_handler.init();

    ros::Rate loop_rate(phidgets_motor_control_hc_handler.publish_frequency_);  //daj to v parametre
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();

        phidgets_motor_control_hc_handler.publish_data();
    }

    phidgets_motor_control_hc_handler.disconnect();
    return 0;
}
