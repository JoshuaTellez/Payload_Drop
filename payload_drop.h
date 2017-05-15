#ifndef MAVLINK_INTERFACE_PAYLOAD_DROP_H
#define MAVLINK_INTERFACE_PAYLOAD_DROP_H

// -------------------------------------------------------------------------------
// Includes
// -------------------------------------------------------------------------------

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include <vector>

using namespace std;

#include "Mavlink/common/mavlink.h"

#include "mavlink_interface.h"
#include "serial_port.h"
#include "UAV_Database.h"


// ------------------------------------------------------------------------------
//   Helper Constants
// ------------------------------------------------------------------------------

#define PI 3.14159265359
#define TO_RADIANS PI / 180;
#define RADIUS_EARTH	6378037.0

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

int main(int argc, char **argv);
int top(int argc, char **argv);

void commands(Mavlink_Interface &autopilot_interface);
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);

/*
 * Quit Handler (Ctrl+C)
 * These objects will reference the actual mavlink and serial objects
 * mavlink_interface_quit = &mavlink_interface
 */
Mavlink_Interface *mavlink_interface_quit;
Serial_Port *serial_port_quit;
void quit_handler( int sig );

// ------------------------------------------------------------------------------
//   Helper Struct
// ------------------------------------------------------------------------------
struct Waypoint{
    double lat;
    double lon;
    int alt;
    Waypoint(double lat = 0, double lon = 0, int alt = 0){
        this->lat = lat;
        this->lon = lon;
        this->alt = alt;
    }
};

// ------------------------------------------------------------------------------
//   Payload Object
// ------------------------------------------------------------------------------

class Payload_Drop{ //Made by Alex Winger and Zach Cheben
private:
    double time;			//The time it takes to get to the point we need to release the payload
    double PLong;			//The longitude of the plane
    double PLat;			//The latitude of the plane
    double velocity;		//The velocity of the plane
    double altitude;		//The alt of the plane
    double targetDistance;	//The distance from the plane to the target
    double dropDistance;	//The distance the payload drops once it is dropped
    double distance;		//The distance from the plane to the point the payload should be dropped

public:

    double TLat;			//The latitude of the target
    double TLong;			//The longitude of the target

    Payload_Drop(double tlat, double tlong, double plat, double plong, double a);

    void setGPSinfo(double plat, double plong, double a);		//Sets the GPS information of the plane
    void setVelocity(int vx, int vy, int vz);

    double timeToDrop(); //Returns the time it will take to reach the drop point

    bool near_target(Mavlink_Interface &mi, const int &radius); //Rudimentary code to drop payload. Use as last resort


    /*
     * Helper Functions
     */
    vector<Waypoint> payload_waypoints(const int &first_distance, const int &second_distance,
                                                     const double &angle); //Returns 3 waypoints
    Waypoint meterDisplacement(const double & deltaX, const double & deltaY, const Waypoint & b);

    double gpsDistance(const double &target_lat, const double &target_long, const double &current_lat,
                       const double &current_long); //Function to convert latitude and longitude into a deistance (Measured in meters)

};



#endif //MAVLINK_INTERFACE_PAYLOAD_DROP_H
