#include "mainwindow.h"
#include "ui_mainwindow.h"



// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &api)
{

    // --------------------------------------------------------------------------
    //   START OFFBOARD MODE
    // --------------------------------------------------------------------------

    api.enable_offboard_control();
    Sleep(1); // give some time to let it sink in

    // now the autopilot is accepting setpoint commands


    // --------------------------------------------------------------------------
    //   SEND OFFBOARD COMMANDS
    // --------------------------------------------------------------------------
    printf("SEND OFFBOARD COMMANDS\n");

    // initialize command data strtuctures
    mavlink_set_position_target_local_ned_t sp;
    mavlink_set_position_target_local_ned_t ip = api.initial_position;

    // autopilot_interface.h provides some helper functions to build the command


    // Example 1 - Set Velocity
//	set_velocity( -1.0       , // [m/s]
//				  -1.0       , // [m/s]
//				   0.0       , // [m/s]
//				   sp        );

    // Example 2 - Set Position
     set_position( ip.x - 5.0 , // [m]
                   ip.y - 5.0 , // [m]
                   ip.z       , // [m]
                   sp         );


    // Example 1.2 - Append Yaw Command
    set_yaw( ip.yaw , // [rad]
             sp     );

    // SEND THE COMMAND
    api.update_setpoint(sp);
    api.write_setpoint();
    // NOW pixhawk will try to move

    // Wait for 8 seconds, check position
    for (int i=0; i < 8; i++)
    {
        mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
        printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
        Sleep(1);
    }

    printf("\n");


    // --------------------------------------------------------------------------
    //   STOP OFFBOARD MODE
    // --------------------------------------------------------------------------

//    api.disable_offboard_control();

    // now pixhawk isn't listening to setpoint commands


    // --------------------------------------------------------------------------
    //   GET A MESSAGE
    // --------------------------------------------------------------------------
    printf("READ SOME MESSAGES \n");

    // copy current messages
    Mavlink_Messages messages = api.current_messages;

    // local position in ned frame
    mavlink_local_position_ned_t pos = messages.local_position_ned;
    printf("Got message LOCAL_POSITION_NED (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
    printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

    // hires imu
    mavlink_highres_imu_t imu = messages.highres_imu;
    printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
    printf("    ap time:     %llu \n", imu.time_usec);
    printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
    printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
    printf("    mag  (NED):  % f % f % f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
    printf("    baro:        %f (mBar) \n"  , imu.abs_pressure);
    printf("    altitude:    %f (m) \n"     , imu.pressure_alt);
    printf("    temperature: %f C \n"       , imu.temperature );

    printf("\n");


    // --------------------------------------------------------------------------
    //   END OF COMMANDS
    // --------------------------------------------------------------------------

    return;

}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    serial_port = new Serial_Port("com3", 57600);
    autopilot = new Autopilot_Interface(serial_port);
    serial_port->open_serial();
    autopilot->start();
    //commands(*autopilot);
    for(int i = 0;i<10;i++){
        autopilot->enable_offboard_control();
        Sleep(1); // give some time to let it sink in
    }


    ui->setupUi(this);
}

MainWindow::~MainWindow()
{

    delete ui;
}
