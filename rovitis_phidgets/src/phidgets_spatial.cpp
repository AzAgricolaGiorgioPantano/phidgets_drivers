/** \brief
 *  \file phidgets_interface_kit_888.hpp
 *  \author Peter Lepej
 *  \date 21.9.2015
 *  \version 1.0
 *  \place: FKBV, Pivola 10, Hoče, Slovenia
 */

#include "rovitis_phidgets/phidgets_spatial.hpp"

int mot_index;
double mot_pose;

namespace phidgets_spatial
{
    /****************************************************************
     * Here we can set up the parameters
     */
    phidgets_spatial::phidgets_spatial(ros::NodeHandle nh)    //constructor for the class
    : nh_(nh),
    spatial(0)
    {
        ros::NodeHandle private_nh("~");
        /* private_nh.param<int>("dev_serial_number", serial_number_, 301820); */ /* test */
        private_nh.param<int>("dev_serial_number", serial_number_, 301820); /* original */
        private_nh.param<std::string>("dev_serial_label", device_label_, string("phidgets_spatial"));
        private_nh.param<std::string>("frame", sp_frame_, string("/phidget_sp"));
        private_nh.param<int>("wait_for_dev_delay", wait_for_dev_delay_, 10000);
        private_nh.param<int>("publish_frequency", publish_frequency_, 5);
        private_nh.param<std::string>("topic_name", topic_name_, string("/phidgets_spatial"));
        private_nh.param<std::string>("set_pose_service_name", service_name_, string("phidgets_spatial_set"));


    }
    /****************************************************************
     *
     */
    phidgets_spatial::~phidgets_spatial()
    {
    }
    /*****************************************************************************************************************
     * Initialization
     */
    void phidgets_spatial::init()
    {
        //connect device
        ROS_INFO("Connecting device ... spatial");
        if (connect_dev())
        {
            ROS_INFO("Connection to spatial successfull!");
            initialised_ = true;
        }
        else
        {
            ROS_ERROR("There is no device spatial connected! Node shutdown!");
            initialised_ = false;

            CPhidget_close((CPhidgetHandle)spatial);
            CPhidget_delete((CPhidgetHandle)spatial);

            ros::shutdown();
        }

        if(initialised_)
        {
            if(serial_number_ == curr_serial_number_)
            {
                //set device label
                const char* cname = device_label_.c_str();
                CPhidget_setDeviceLabel((CPhidgetHandle)spatial, cname);

                //create topics to publish data
                pub_ = nh_.advertise<rovitis_phidgets::phidget_spatial>(topic_name_, 1);
                pub_imu_ = nh_.advertise<sensor_msgs::Imu>("imu_phidgets/data", 1);

                // **** advertise topics
                cal_publisher_ = nh_.advertise<std_msgs::Bool>("imu_phidgets/is_calibrated", 5);

                // **** advertise services
                cal_srv_ = nh_.advertiseService("imu_phidgets/calibrate", &phidgets_spatial::calibrateService, this);

                // **** initialize variables
                last_imu_time_ = time_now;
                time_zero_ = ros::Time::now();


            }else
            {
                ROS_ERROR("Defined device serial number: %i dose not match to current connected device serial number: %i!", serial_number_,curr_serial_number_);
            }
        }
    }

	//callback that will run if the Spatial is attached to the computer
	int CCONV AttachHandler(CPhidgetHandle spatial, void *userptr)
	{
		int serialNo;
		CPhidget_getSerialNumber(spatial, &serialNo);
		printf("Spatial %10d attached!", serialNo);

		return 0;
	}

	//callback that will run if the Spatial is detached from the computer
	int CCONV DetachHandler(CPhidgetHandle spatial, void *userptr)
	{
		int serialNo;
		CPhidget_getSerialNumber(spatial, &serialNo);
		printf("Spatial %10d detached! \n", serialNo);

		return 0;
	}

	//callback that will run if the Spatial generates an error
	int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown)
	{
		printf("Error handled. %d - %s \n", ErrorCode, unknown);
		return 0;
	}

    bool phidgets_spatial::calibrateService(std_srvs::Empty::Request  &req,
                                   std_srvs::Empty::Response &res)
    {
      calibrate();
      return true;
    }

    void phidgets_spatial::calibrate()
    {
      ROS_INFO("Calibrating IMU...");
      //zero();
      ROS_INFO("Calibrating IMU done.");

      time_zero_ = ros::Time::now();

      // publish message
      std_msgs::Bool is_calibrated_msg;
      is_calibrated_msg.data = true;
      cal_publisher_.publish(is_calibrated_msg);
    }


	//callback that will run at datarate
	//data - array of spatial event data structures that holds the spatial data packets that were sent in this event
	//count - the number of spatial data event packets included in this event
    int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
	{

        
		int i;
		printf("Number of Data Packets in this event: %d\n", count);
		for(i = 0; i < count; i++)
		{
			printf("=== Data Set: %d ===\n", i);
			printf("Acceleration> x: %6f  y: %6f  x: %6f\n", data[i]->acceleration[0], data[i]->acceleration[1], data[i]->acceleration[2]);
			printf("Angular Rate> x: %6f  y: %6f  x: %6f\n", data[i]->angularRate[0], data[i]->angularRate[1], data[i]->angularRate[2]);
			printf("Magnetic Field> x: %6f  y: %6f  x: %6f\n", data[i]->magneticField[0], data[i]->magneticField[1], data[i]->magneticField[2]);
			printf("Timestamp> seconds: %d -- microseconds: %d\n", data[i]->timestamp.seconds, data[i]->timestamp.microseconds);
		}

		printf("---------------------------------------------\n");
        

        if(!initialised_)
        {
            return 0;
        }

        for(int i = 0; i < count; i++)
        {

            
            // **** calculate time from timestamp
            ros::Duration time_imu(data[i]->timestamp.seconds +
                                   data[i]->timestamp.microseconds * 1e-6);

            ros::Time time_now = time_zero_ + time_imu;

            double timediff = time_now.toSec() - ros::Time::now().toSec();
            if (fabs(timediff) > 0.1) {
              ROS_WARN("IMU time lags behind by %f seconds, resetting IMU time offset!", timediff);
              time_zero_ = ros::Time::now() - time_imu;
              time_now = ros::Time::now();
            }

            // **** initialize if needed

            if (!initialized_)
            {
              last_imu_time_ = time_now;
              initialized_ = true;
            }


            imu_header_time = time_now;
            


            // set linear acceleration
            imu_linear_acceleration_x = - data[i]->acceleration[0] * G;
            imu_linear_acceleration_y = - data[i]->acceleration[1] * G;
            imu_linear_acceleration_z = - data[i]->acceleration[2] * G;

            // set angular velocities
            imu_angular_velocity_x = data[i]->angularRate[0] * (M_PI / 180.0);
            imu_angular_velocity_y = data[i]->angularRate[1] * (M_PI / 180.0);
            imu_angular_velocity_z = 0;
            if (abs(imu_angular_velocity_z_old = data[i]->angularRate[2] * (M_PI / 180.0)) > 0.009)
                imu_angular_velocity_z = data[i]->angularRate[2] * (M_PI / 180.0);

            imu_angular_velocity_z_old = data[i]->angularRate[2] * (M_PI / 180.0);

            if (data[i]->magneticField[0] != PUNK_DBL)
            {
              mag_vector_x = data[i]->magneticField[0];
              mag_vector_y = data[i]->magneticField[1];
              mag_vector_z = data[i]->magneticField[2];
            }
            else
            {
              double nan = std::numeric_limits<double>::quiet_NaN();

              mag_vector_x = nan;
              mag_vector_y = nan;
              mag_vector_z = nan;
            }

        }
        return 0;

	}

    void phidgets_spatial::display_generic_properties(CPhidgetHandle phid)
    {
        int serialNo, version;
        const char* ptr;
        int numAccelAxes, numGyroAxes, numCompassAxes, dataRateMax, dataRateMin;

        CPhidget_getDeviceType(phid, &ptr);
        CPhidget_getSerialNumber(phid, &serialNo);
        CPhidget_getDeviceVersion(phid, &version);
        CPhidgetSpatial_getAccelerationAxisCount((CPhidgetSpatialHandle)phid, &numAccelAxes);
        CPhidgetSpatial_getGyroAxisCount((CPhidgetSpatialHandle)phid, &numGyroAxes);
        CPhidgetSpatial_getCompassAxisCount((CPhidgetSpatialHandle)phid, &numCompassAxes);
        CPhidgetSpatial_getDataRateMax((CPhidgetSpatialHandle)phid, &dataRateMax);
        CPhidgetSpatial_getDataRateMin((CPhidgetSpatialHandle)phid, &dataRateMin);

        curr_serial_number_ = serialNo;

        printf("%s\n", ptr);
        printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
        printf("Number of Accel Axes: %i\n", numAccelAxes);
        printf("Number of Gyro Axes: %i\n", numGyroAxes);
        printf("Number of Compass Axes: %i\n", numCompassAxes);
        printf("datarate> Max: %d  Min: %d\n", dataRateMax, dataRateMin);


    }

    bool phidgets_spatial::connect_dev()
    {

        int result;
        const char *err;

        //Declare a spatial handle
        CPhidgetSpatialHandle spatial = 0;

        //create the spatial object
        CPhidgetSpatial_create(&spatial);

        //Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
        CPhidget_set_OnAttach_Handler((CPhidgetHandle)spatial, AttachHandler, NULL);
        CPhidget_set_OnDetach_Handler((CPhidgetHandle)spatial, DetachHandler, NULL);
        CPhidget_set_OnError_Handler((CPhidgetHandle)spatial, ErrorHandler, NULL);

        //Registers a callback that will run according to the set data rate that will return the spatial data changes
        //Requires the handle for the Spatial, the callback handler function that will be called,
        //and an arbitrary pointer that will be supplied to the callback function (may be NULL)
        CPhidgetSpatial_set_OnSpatialData_Handler(spatial, SpatialDataHandler, NULL);

        //open the spatial object for device connections
        CPhidget_open((CPhidgetHandle)spatial, serial_number_);

        //get the program to wait for a spatial device to be attached
        printf("Waiting for spatial to be attached.... \n");
        if((result = CPhidget_waitForAttachment((CPhidgetHandle)spatial, wait_for_dev_delay_)))
        {
            CPhidget_getErrorDescription(result, &err);
            printf("Problem waiting for attachment: %s\n", err);
            return 0;
        }

        //Set the data rate for the spatial events
        CPhidgetSpatial_setDataRate(spatial, 16);

        //Display the properties of the attached spatial device
        display_generic_properties((CPhidgetHandle)spatial);
        return true;

    }

    void phidgets_spatial::publish_data()
    {


        //publish data
        rovitis_phidgets::phidget_spatial data;

        data.header.frame_id = sp_frame_;
        data.header.stamp = ros::Time::now();

        data.imu_msg_linear_acceleration_x = imu_linear_acceleration_x;
        data.imu_msg_linear_acceleration_y = imu_linear_acceleration_y;
        data.imu_msg_linear_acceleration_z = imu_linear_acceleration_z;
        data.imu_msg_angular_velocity_x = imu_angular_velocity_x;
        data.imu_msg_angular_velocity_y = imu_angular_velocity_y;
        data.imu_msg_angular_velocity_z = imu_angular_velocity_z;
        data.mag_msg_vector_x = mag_vector_x;
        data.mag_msg_vector_y = mag_vector_y;
        data.mag_msg_vector_z = mag_vector_z;

        pub_.publish(data);

        sensor_msgs::Imu imu_data;

        imu_data.header.frame_id = sp_frame_;
        imu_data.header.stamp = ros::Time::now();

        imu_data.linear_acceleration.x = imu_linear_acceleration_x;
        imu_data.linear_acceleration.y = imu_linear_acceleration_y;
        imu_data.linear_acceleration.z = imu_linear_acceleration_z;
        imu_data.angular_velocity.x = imu_angular_velocity_x;
        imu_data.angular_velocity.y = imu_angular_velocity_y;
        imu_data.angular_velocity.z = imu_angular_velocity_z;

        tf::Quaternion q;
        q.setRPY(mag_vector_x, mag_vector_y, mag_vector_z);

        imu_data.orientation.x = q.x();
        imu_data.orientation.y = q.y();
        imu_data.orientation.z = q.z();
        imu_data.orientation.w = q.w();

        pub_imu_.publish(imu_data);

    }

    void phidgets_spatial::disconnect()
    {
        ROS_INFO("Closing...");
        CPhidget_close((CPhidgetHandle)spatial);
        CPhidget_delete((CPhidgetHandle)spatial);
    }

//    void phidgets_spatial::velocityCommandCallback(const geometry_msgs::Twist::ConstPtr& ptr)
//    {

//        ROS_INFO_ONCE("Velocity Command Received! Cmd Vel is published at same frequency as input /cmd_vel topic!");

//        geometry_msgs::Twist cmd = *ptr;
//        current_linear_velocity_ = cmd.linear.x;
//        current_angular_velocity_ = cmd.angular.z;

//        if ((current_angular_velocity_!=0) || (current_angular_velocity_!=0))
//        {
//            ROS_INFO("cmd_vel %.3f angular %.3f", current_linear_velocity_, current_angular_velocity_);
//        }


}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "phidgets_spatial");
    ros::NodeHandle nh;

    //For node registering
    phidgets_spatial::phidgets_spatial phidgets_spatial_handler(nh);

    //initialization
    phidgets_spatial_handler.init();

    ros::Rate loop_rate(phidgets_spatial_handler.publish_frequency_);  //daj to v parametre
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();

        phidgets_spatial_handler.publish_data();
    }

    phidgets_spatial_handler.disconnect();
    return 0;
}
