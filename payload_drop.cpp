/*
 * NGCP - UAV - Avionics
 * Purpose - Autonomously drop a payload on a given target using mavlink
 *
 * @author - Joshua Tellez <joshtel@live.com>
 *
 */

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include "payload_drop.h"



// ------------------------------------------------------------------------------
//   TOP -- essentially main. main() calls this function
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{
    /*
     * Config variables
     */
    int wp_radius;
    int servo, pwm;
    int first_distance, second_distance;
    double angle;
    int target_radius;
    bool database;
    double target_lat, target_long;
    string port;

    /*
     * Preprocessing variables in config file
     */
    string line;
    ifstream config ("config.cfg");
    if(config.is_open()){
        int i = 1;
        while(getline(config, line)){
            switch (i++) {
                case 1: //wp_radius of waypoints
                {
                    wp_radius = stoi(line);
                    printf("WP_Radius: %d\n", wp_radius);
                    break;
                }
                case 2: //servo number
                {
                    servo = stoi(line);
                    printf("Servo: %d\n", servo);
                    break;
                }
                case 3: //pwm value
                {
                    pwm = stoi(line);
                    printf("PWM: %d\n",pwm);
                    break;
                }
                case 4: //first_distance away from target
                {
                    first_distance = stoi(line);
                    printf("First Distance: %d\n",first_distance);
                    break;
                }
                case 5:
                {
                    second_distance = stoi(line);
                    printf("Second Distance: %d\n",second_distance);
                    break;
                }
                case 6: //angle
                {
                    angle = stof(line);
                    printf("Angle: %f\n",angle);
                    break;
                }
                case 7: //target wp_radius
                {
                    target_radius = stoi(line);
                    printf("Target Radius: %d\n",target_radius);
                    break;
                }
                case 8: //O to turn of database, 1 to turn on database
                {
                    database = stoi(line);
                    printf("Database : %d\n",database);
                    break;
                }
                case 9:
                {
                    target_lat = stof(line);
                    printf("Target Lat: %f\n",target_lat);
                    break;
                }
                case 10:
                {
                    target_long = stof(line);
                    printf("Target Long: %f\n",target_long);
                    break;
                }
                case 11: //Name of port pixhawk connects to
                {
                    port = line;
                    break;
                }
            }
        }
    }
    else{
        printf("ERROR: Could not open config file\n");
    }

    // --------------------------------------------------------------------------
    //   PARSE THE COMMANDS -- TODO
    // --------------------------------------------------------------------------

    // Default input argument
    char *uart_name = (char*) port.c_str();
    int baudrate = 57600;

    // do the parse, will throw an int if it fails
  //  parse_commandline(argc, argv, uart_name, baudrate);


    // --------------------------------------------------------------------------
    //   PORT and THREAD STARTUP
    // --------------------------------------------------------------------------

    /*
     * Instantiate a serial port object
     *
     * This object handles the opening and closing of the offboard computer's
     * serial port over which it will communicate to an autopilot.  It has
     * methods to read and write a mavlink_message_t object.  To help with read
     * and write in the context of pthreading, it guards port operations with a
     * pthread mutex lock.
     *
     */
    Serial_Port serial_port(uart_name, baudrate);


    /*
     * Instantiate an autopilot interface object
     *
     * This starts two threads for read and write over MAVlink. The read thread
     * listens for any MAVlink message and pushes it to the current_messages
     * attribute.  The write thread at the moment only streams a position target
     * in the local NED frame (mavlink_set_position_target_local_ned_t), which
     * is changed by using the method update_setpoint().  Sending these messages
     * are only half the requirement to get response from the autopilot, a signal
     * to enter "offboard_control" mode is sent by using the enable_offboard_control()
     * method.  Signal the exit of this mode with disable_offboard_control().  It's
     * important that one way or another this program signals offboard mode exit,
     * otherwise the vehicle will go into failsafe.
     *
     */
    Mavlink_Interface mavlink_interface(&serial_port);

    /*
     * Setup interrupt signal handler
     *
     * Responds to early exits signaled with Ctrl-C.  The handler will command
     * to exit offboard mode if required, and close threads and the port.
     * The handler in this example needs references to the above objects.
     *
     */
    serial_port_quit         = &serial_port;
    mavlink_interface_quit = &mavlink_interface;
    signal(SIGINT,quit_handler);

    /*
     * Start the port and autopilot_interface
     * This is where the port is opened, and read and write threads are started.
     */
    serial_port.start();


    /*
     * Set messages to read
     */
    mavlink_interface.messages_to_read.read_global_position_int = true;
    mavlink_interface.messages_to_read.read_heartbeat = true;
    mavlink_interface.messages_to_read.read_battery_status = true;

    /*
     * Start reading messages
     */
    mavlink_interface.start();


    /*
     * Polling target location --
     */

    if(database) { //If database is on
        UAV_DatabaseConnect("plane1", "root", "ngcp"); //Connect to the database
        double target[3]; //target: {alt, lat, long}
        int startMission = UAV_CheckMissionStatus();
        while (startMission != 1) {
            printf("Waiting for mission to start\n");
            //Updating our GPS location
            UAV_InsertGPS_LOCAL(mavlink_interface.current_messages.global_position_int.alt / 1E3,
                                mavlink_interface.current_messages.global_position_int.lat / 1E7,
                                mavlink_interface.current_messages.global_position_int.lon / 1E7);
            startMission = UAV_CheckMissionStatus();
            usleep(1000000); //check and update once a second
        }
        printf("Starting Mission!!!!!!\n");

        bool recievedTarget = false;

        while (!recievedTarget){ //While we have not recieved target

            //Checking if database has target
            UAV_PullTARGET_LOCAL(target);
            usleep(500000); //check and update 2 times a second
            if(target[1] < 50 && target[1] > 0){ //If we have a 'reasonable' lat
                recievedTarget = true;
            }
        }

        target_lat = target[1];
        target_long = target[2];

        printf("Target Recieved!!!!!\n");
        printf("Target Lat: %f, Target Long: %f\n", target_lat, target_long);

    }

    double plane_lat = mavlink_interface.current_messages.global_position_int.lat / 1E7,
            plane_long = mavlink_interface.current_messages.global_position_int.lon / 1E7,
            plane_alt = mavlink_interface.current_messages.global_position_int.alt / 1E3;


    /*
     * End of TODO Cady's code
     */




    /*
     * Payload_drop object
     */
    Payload_Drop payload_drop(target_lat, target_long, plane_lat, plane_long, plane_alt);
    payload_drop.setVelocity(mavlink_interface.current_messages.global_position_int.vx,  // X velocity * 100 (m/s)
                             mavlink_interface.current_messages.global_position_int.vy, // Y velocity * 100 (m/s)
                             mavlink_interface.current_messages.global_position_int.vz); // z velocity * 100 (m/s)



    /*
     * Create 3 waypoints at an angle
     *
     *    (First_wp)--------100m--------(target_wp)---30m---(Third_wp)
     *
     */
    vector<Waypoint> payload_waypoints = payload_drop.payload_waypoints(first_distance,second_distance,angle);


    vector<mavlink_mission_item_t> waypoints;
    // For some reason WP 0(HOME) needs to be sent but need different command to actually change HOME ???
    waypoints.push_back(mavlink_interface.create_waypoint(0,0,0,0)); //Dummy data that is required* but not used
    waypoints.push_back(mavlink_interface.create_waypoint(payload_waypoints[0].lat, payload_waypoints[0].lon,100,1,wp_radius));
    waypoints.push_back(mavlink_interface.create_waypoint(payload_waypoints[1].lat, payload_waypoints[1].lon,100,2,wp_radius));
    waypoints.push_back(mavlink_interface.create_waypoint(payload_waypoints[2].lat, payload_waypoints[2].lon,100,3,wp_radius));


    /*
     * Send waypoints to pixhawk
     */
    mavlink_interface.write_waypoints(waypoints);

    /*
     * Staring Payload mission
     */

    //We must first get to first_wp
    bool at_first_wp = false;
    while (!at_first_wp) {
        usleep(200000); //check and update gps 5 times a second
        //Updating our GPS location
        if(database) {
            UAV_InsertGPS_LOCAL(mavlink_interface.current_messages.global_position_int.alt / 1E3,
                                mavlink_interface.current_messages.global_position_int.lat / 1E7,
                                mavlink_interface.current_messages.global_position_int.lon / 1E7);
        }

        double distance = payload_drop.gpsDistance(payload_waypoints[0].lat, payload_waypoints[0].lon, //While we are not at first_wp
                                   mavlink_interface.current_messages.global_position_int.lat / 1E7,
                                   mavlink_interface.current_messages.global_position_int.lon / 1E7);
        printf("Distance to first wp: %f\n",distance);


        if(distance < wp_radius){
            at_first_wp = true;
        }
    }

    printf("Reached first waypoint!!!\n");


    /*
     * check to see if we are near target then drop payload
     */
    bool nearTarget = false;
    while(!nearTarget){ //while we are not near the target

        usleep(100000); //check and update gps 10 times a second
        //Updating our GPS location
        if(database){
            UAV_InsertGPS_LOCAL(mavlink_interface.current_messages.global_position_int.alt / 1E3,
                                mavlink_interface.current_messages.global_position_int.lat / 1E7,
                                mavlink_interface.current_messages.global_position_int.lon / 1E7);
        }

        double distance = payload_drop.gpsDistance(payload_drop.TLat, payload_drop.TLong, //While we are not at first_wp
                                                   mavlink_interface.current_messages.global_position_int.lat / 1E7,
                                                   mavlink_interface.current_messages.global_position_int.lon / 1E7);
        printf("Distance to first target: %f\n",distance);

        if(distance < target_radius){
            nearTarget = true;
            mavlink_interface.write_set_servo(servo,pwm);
        }
    }

    printf("Dropped Payload!!!!!\n");
    if(database){
        while(true){
            usleep(500000); //check and update gps 2 times a second
            //Updating our GPS location
            UAV_InsertGPS_LOCAL(mavlink_interface.current_messages.global_position_int.alt / 1E3,
                                mavlink_interface.current_messages.global_position_int.lat / 1E7,
                                mavlink_interface.current_messages.global_position_int.lon / 1E7);
        }
    }




    // --------------------------------------------------------------------------
    //   THREAD and PORT SHUTDOWN
    // --------------------------------------------------------------------------

    /*
     * Now that we are done we can stop the threads and close the port
     */
    mavlink_interface.stop();
    serial_port.stop();


    // --------------------------------------------------------------------------
    //   DONE
    // --------------------------------------------------------------------------

    // woot!
    return 0;

}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");

    try {
        mavlink_interface_quit->handle_quit(sig);
    }
    catch (int error){}

    // serial port
    try {
        serial_port_quit->handle_quit(sig);
    }
    catch (int error){}

    // end program here
    exit(0);

}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
    // This program uses throw, wrap one big try/catch here
    try
    {
        int result = top(argc,argv);
        return result;
    }

    catch ( int error )
    {
        fprintf(stderr,"payload_drop threw exception %i \n" , error);
        return error;
    }

}


// ------------------------------------------------------------------------------
//   Payload Drop Object
// ------------------------------------------------------------------------------


/*
 * Constructor
 */
Payload_Drop::
Payload_Drop(double tlat, double tlong, double plat, double plong, double a) {
    this->TLat = tlat;
    this->TLong = tlong;
    PLat = plat;
    PLong = plong;
    altitude = a;
}
/*
 * Change plane GPS( lat, lon, alt)
 */
void
Payload_Drop::
setGPSinfo(double plat, double plong, double a)
{
    PLat = plat;
    PLong = plong;
    altitude = a;
}

/*
 * Calculates time it will take to drop target -- TODO input code but first fix Douglas' NGCPPayloadDrop.cpp
 */
double
Payload_Drop::
timeToDrop()
{
    return 0;
}

/*
 * Set Velocity given vector components
 */
void
Payload_Drop::
setVelocity(int vx, int vy, int vz)
{
    velocity = sqrt(vx*vx) + sqrt(vy*vy) + sqrt(vz*vz);
}


/*
 * Rudimentary code to drop the payload when we are near the target. Last resort if timeToDrop is not working
 * @param radius: in meters how close do we get to target before dropping payload
 */
bool
Payload_Drop::
near_target(Mavlink_Interface &mi, const int &radius) {
    bool nearTarget = false;
    printf("Radius: %d\n", radius);
    int i = 0;
    while(!nearTarget){
        //Update GPS
        setGPSinfo(mi.current_messages.global_position_int.lat / 1E7,
                   mi.current_messages.global_position_int.lon / 1E7,
                   mi.current_messages.global_position_int.relative_alt / 1E3);
        double dist = gpsDistance(TLat, TLong, PLat, PLong);
        printf("Distance to target: %f\n",dist);
        if(radius >= dist) break;
        usleep(1000000);
    }
    return true;




}

// --------------------------------------------------------------------------------------
// Helper Functions for payload drop
// --------------------------------------------------------------------------------------

/*
	Calculates the distance between the target's latitude and longitude coordinates and the plane's latitude and longitude coordinates
	Calculates in meters
*/
double
Payload_Drop::
gpsDistance(const double &target_lat, const double &target_long, const double &current_lat, const double &current_long)
{
    double tLatRad = target_lat * TO_RADIANS;
    double pLatRad = current_lat * TO_RADIANS;
    double deltaLat = (current_lat - target_lat) * TO_RADIANS;
    double deltaLong = (current_long - target_long) * TO_RADIANS;

    double var = sin(deltaLat / 2) * sin(deltaLat / 2) + (cos(tLatRad) * cos(pLatRad)) * (sin(deltaLong / 2) * sin(deltaLong / 2));
    double c = 2 * atan2(sqrt(var), sqrt(1 - var));

    return RADIUS_EARTH * c;
}

/*
 * Return the three waypoints needed to properly drop payload
 */
vector<Waypoint>
Payload_Drop::
payload_waypoints(const int &first_distance, const int &second_distance, const double &angle)
{
    //Initialize data strucutres
    vector<Waypoint> waypoints;
    Waypoint target(TLat,TLong,altitude);
    double angle_rad = angle * TO_RADIANS;

    //Find displacement of first waypoint. Think of it like a right triangle and we're finding the 'a' and 'b' sides
    double first_x_displacement = first_distance * cos(angle_rad);
    double first_y_displacement = first_distance * sin(angle_rad);

    //Find the waypoint given the x and y displacement of target
    Waypoint first_waypoint = meterDisplacement(first_x_displacement,first_y_displacement, target);
    waypoints.push_back(first_waypoint);

    //Our target is the middle waypoint
    waypoints.push_back(target);

    //Find displacement of first waypoint. Think of it like a right triangle and we're finding the 'a' and 'b' sides
    double second_x_displacement = -second_distance * cos(angle_rad);
    double second_y_displacement = -second_distance * sin(angle_rad);
    //Find the waypoint given the x and y displacement of target
    Waypoint second_waypoint = meterDisplacement(second_x_displacement,second_y_displacement, target);
    waypoints.push_back(second_waypoint);

    return waypoints;

}

/*Return a new position given the x and y displacement in meters of a waypoint*/
Waypoint
Payload_Drop::
meterDisplacement(const double & deltaX, const double & deltaY, const Waypoint & b){
    //coordinate offset in Radians
    float deltaLat = (deltaY / RADIUS_EARTH);
    float deltaLong = deltaX / (RADIUS_EARTH * cos(b.lat * PI / 180));

    Waypoint newPosition;
    newPosition.lat = b.lat + (deltaLat * (180 / PI));
    newPosition.lon = b.lon + deltaLong * ((180 / PI));
    return newPosition;
}
