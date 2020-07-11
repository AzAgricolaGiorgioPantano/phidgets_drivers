// GPSTest.cpp : Defines the entry point for the console application.
//


#include "rovitis_phidgets/phidgets_gps.hpp"

namespace phidgets_gps
{
    /****************************************************************
     * Here we can set up the parameters
     */
    phidgets_gps::phidgets_gps(ros::NodeHandle nh)    //constructor for the class
    : nh_(nh),
    gps(0)
    {
        ros::NodeHandle private_nh("~");
        private_nh.param<int>("dev_serial_number", serial_number_, 131340);
        private_nh.param<std::string>("dev_serial_label", device_label_, string("phidgets_gps"));
        private_nh.param<std::string>("frame", sp_frame_, string("/phidget_gps"));
        private_nh.param<int>("wait_for_dev_delay", wait_for_dev_delay_, 10000);
        private_nh.param<int>("publish_frequency", publish_frequency_, 5);
        private_nh.param<std::string>("topic_name", topic_name_, string("/phidgets_gps"));


    }
    /****************************************************************
     *
     */
    phidgets_gps::~phidgets_gps()
    {
    }
    /************************************************************************************************
     * Initialization
     */

    void phidgets_gps::init()
    {
        //connect device
        ROS_INFO("Connecting device ... gps");
        if (connect_dev())
        {
            ROS_INFO("Connection to gps successfull!");
            initialised_ = true;
        }
        else
        {
            ROS_ERROR("There is no device gps connected! Node shutdown!");
            initialised_ = false;

            CPhidget_close((CPhidgetHandle)gps);
            CPhidget_delete((CPhidgetHandle)gps);

            ros::shutdown();
        }

        if(initialised_)
        {
            if(serial_number_ == curr_serial_number_)
            {
                //set device label
                const char* cname = device_label_.c_str();
                CPhidget_setDeviceLabel((CPhidgetHandle)gps, cname);

                //create topics to publish data
                pub_gps_ = nh_.advertise<rovitis_phidgets::phidget_gps>(topic_name_, 1);



            }else
            {
                ROS_ERROR("Defined device serial number: %i dose not match to current connected device serial number: %i!", serial_number_,curr_serial_number_);
            }
        }
    }

	int CCONV AttachHandler(CPhidgetHandle phid, void *userptr)
	{
		GPSTime time;
		CPhidgetGPSHandle gps = (CPhidgetGPSHandle)phid;
		if(!CPhidgetGPS_getTime(gps, &time))
			printf("Attach handler ran at: %02d:%02d:%02d.%03d\n", time.tm_hour, time.tm_min, time.tm_sec, time.tm_ms);
		else
			printf("Attach handler ran!\n");
		return 0;
	}

	int CCONV DetachHandler(CPhidgetHandle phid, void *userptr)
	{
		printf("Detach handler ran!\n");
		return 0;
	}

	int CCONV ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *unknown)
	{
		printf("Error handler ran!\n");
		return 0;
	}

	int CCONV posnChange(CPhidgetGPSHandle phid, void *userPtr, double latitude, double longitude, double altitude)
	{
		GPSDate date;
		GPSTime time;
		CPhidgetGPSHandle gps = (CPhidgetGPSHandle)phid;
		double heading, velocity;

        //printf("Position Change event: lat: %3.4lf, long: %4.4lf, alt: %5.4lf\n", latitude, longitude, altitude);
		if(!CPhidgetGPS_getDate(gps, &date) && !CPhidgetGPS_getTime(gps, &time))
        {
            //printf(" Date: %02d/%02d/%02d Time %02d:%02d:%02d.%03d\n", date.tm_mday, date.tm_mon, date.tm_year, time.tm_hour, time.tm_min, time.tm_sec, time.tm_ms);
        }

        std::stringstream ss;
        const char* msg;

        ss.str("");

        ss << date.tm_year << ":";
        ss << date.tm_mon << ":";
        ss << date.tm_mday << "-";
        ss << time.tm_hour << ":";
        ss << time.tm_min << ":";
        ss << time.tm_sec << ":";
        ss << time.tm_ms << std::endl;

        gps_datetime =  ss.str();

		if(!CPhidgetGPS_getHeading(gps, &heading) && !CPhidgetGPS_getVelocity(gps, &velocity))
        {
            //printf(" Heading: %3.2lf, Velocity: %4.3lf\n",heading, velocity);
        }

        gps_latitude = latitude;
        gps_longitude = longitude;
        gps_altitude = altitude;
        gps_heading = heading;
        gps_velocity = velocity;

		return 0;
	}

	int CCONV fixChange(CPhidgetGPSHandle phid, void *userPtr, int status)
	{
        gps_status = status;
		printf("Fix change event: %d\n", status);
		return 0;
	}

        void phidgets_gps::disconnect()
        {
	    ROS_INFO("Closing...");
    	    CPhidget_close((CPhidgetHandle)gps);
	    CPhidget_delete((CPhidgetHandle)gps);
        }


	void phidgets_gps::display_generic_properties(CPhidgetHandle phid)
	{
		int sernum, version;
		const char *deviceptr;
		CPhidget_getDeviceType(phid, &deviceptr);
		CPhidget_getSerialNumber(phid, &sernum);
		CPhidget_getDeviceVersion(phid, &version);

        curr_serial_number_ = sernum;

		printf("%s\n", deviceptr);
		printf("Version: %8d SerialNumber: %10d\n", version, sernum);
		return;
	}

    void phidgets_gps::publish_data()
    {

        //publish data
        rovitis_phidgets::phidget_gps data;

        data.header.frame_id = sp_frame_;
        data.header.stamp = ros::Time::now();

        data.latitude = gps_latitude;
        data.longitude = gps_longitude;
        data.altitude = gps_altitude;
        data.heading = gps_heading;
        data.velocity = gps_velocity;
        data.datetime = gps_datetime;
        data.status = gps_status;

        pub_gps_.publish(data);

    }

    bool phidgets_gps::connect_dev()
    {

		int result;
		CPhidgetGPSHandle gps;
		//CPhidget_enableLogging(PHIDGET_LOG_VERBOSE, NULL);

		CPhidgetGPS_create(&gps);

		CPhidget_set_OnAttach_Handler((CPhidgetHandle)gps, AttachHandler, NULL);
		CPhidget_set_OnDetach_Handler((CPhidgetHandle)gps, DetachHandler, NULL);
		CPhidget_set_OnError_Handler((CPhidgetHandle)gps, ErrorHandler, NULL);

		CPhidgetGPS_set_OnPositionChange_Handler(gps, posnChange, NULL);
		CPhidgetGPS_set_OnPositionFixStatusChange_Handler(gps, fixChange, NULL);

		CPhidget_open((CPhidgetHandle)gps, serial_number_);

		//Wait for 10 seconds, otherwise exit
		if(result = CPhidget_waitForAttachment((CPhidgetHandle)gps, 10000))
		{
			const char *err;
			CPhidget_getErrorDescription(result, &err);
			printf("Problem waiting for attachment: %s\n", err);
            return 0;
		}

		display_generic_properties((CPhidgetHandle)gps);


        return true;
	}

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "phidgets_gps");
    ros::NodeHandle nh;

    //For node registering
    phidgets_gps::phidgets_gps phidgets_gps_handler(nh);

    //initialization
    phidgets_gps_handler.init();

    ros::Rate loop_rate(phidgets_gps_handler.publish_frequency_);  //daj to v parametre
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();

        phidgets_gps_handler.publish_data();
    }

    phidgets_gps_handler.disconnect();
    return 0;
}
