/** \brief
 *  \file phidgets_interface_kit_888.hpp
 *  \author Peter Lepej
 *  \date 21.9.2015
 *  \version 1.0
 *  \place: FKBV, Pivola 10, Hoče, Slovenia
 */

#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <sstream>

//phidgets stuff
#include <libphidgets/phidget21.h>
//#include <phidgets_api/phidget.h>

// This error handler can handle any CPhidget function that returns an int
int LocalErrorCatcher (int errorCode) {

    const char *errorDescription;

    // If the error code is 0, everything is okay
    if (errorCode != 0) {

        // Otherwise, you can print specific messages or perform actions by error value.
        switch (errorCode) {
           default:
               printf("Error: An error occurred with code %d.\n", errorCode);

               LocalErrorCatcher(
                   CPhidget_getErrorDescription (errorCode, &errorDescription));
               printf("The description for this error is: %s\n", errorDescription);
               break;
       }
    }
    return 0;
}

int CCONV AttachHandler (CPhidgetHandle device, void *userptr) {

    int serialNumber;
    const char *name;

    LocalErrorCatcher(
        CPhidget_getDeviceName(device, &name));
    LocalErrorCatcher(
        CPhidget_getSerialNumber(device, &serialNumber));

    printf("Hello Device %s, Serial Number: %d\n", name, serialNumber);

    return 0;
}

int CCONV DetachHandler (CPhidgetHandle device, void *userptr) {

    int serialNumber;
    const char *name;

    LocalErrorCatcher(
        CPhidget_getDeviceName(device, &name));
    LocalErrorCatcher(
        CPhidget_getSerialNumber(device, &serialNumber));

    printf("Goodbye Device %s, Serial Number: %d\n", name, serialNumber);

    return 0;
}

// When using an error handler with the manager, it takes a
// CPhidgetManagerHandle, when using an individual object, the
// object serves as its own handle.
int CCONV LibraryErrorHandler (CPhidgetManagerHandle device, void *usrptr,
                    int errorCode, const char *errorDescription) {
    printf("Error Event phidgets_list_devices: %d - %s\n", errorCode, errorDescription);
    return 0;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "phidgets_list_devices");
    ros::NodeHandle nh;

    int result;
    const char *err;

    CPhidgetManagerHandle device = 0;
    LocalErrorCatcher(
        CPhidgetManager_create(&device));

    LocalErrorCatcher(
        CPhidgetManager_set_OnAttach_Handler((CPhidgetManagerHandle) device,
                                         AttachHandler, NULL));

    LocalErrorCatcher(
        CPhidgetManager_set_OnDetach_Handler((CPhidgetManagerHandle ) device,
                                         DetachHandler, NULL));

    LocalErrorCatcher(
        CPhidgetManager_set_OnError_Handler((CPhidgetManagerHandle) device,
                                        LibraryErrorHandler, NULL));
    printf("Opening...\n");
    // Most opening and closing would be via a cast to
    // (CPhidgetHandle), however, this manager has its
    // own handle struct to cast to.
    LocalErrorCatcher(
        CPhidgetManager_open((CPhidgetManagerHandle) device));

    printf("Phidget Simple Playground (plug and unplug devices)\n");
    printf("Press Enter to end anytime...\n");
    getchar();

    printf("Closing...\n");
    LocalErrorCatcher(
        CPhidgetManager_close((CPhidgetManagerHandle) device));
    LocalErrorCatcher(
        CPhidgetManager_delete((CPhidgetManagerHandle) device));

    return 0;


    //ros::spin();
    return 0;
}
