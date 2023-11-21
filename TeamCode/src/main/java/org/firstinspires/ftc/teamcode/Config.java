package org.firstinspires.ftc.teamcode;

public class Config {
    //////////////////////
    //VARIABLES
    //////////////////////
    /*
    Robot WIFI: 22556-RC OR 22556-B-RC
    WIFI Password: BHS22556
    IP Address: http://192.168.43.1:8080/
     */

    // Drivetrain
    public static volatile String DRIVE_FRONT_LEFT = "drive_front_left";
    public static volatile String DRIVE_FRONT_RIGHT = "drive_front_right";
    public static volatile String DRIVE_BACK_LEFT = "drive_back_left";
    public static volatile String DRIVE_BACK_RIGHT = "drive_back_right";
    public static volatile double ENCODER_RESOLUTION = 537.7;
    // 9.6 cm small, 14.0 cm big
    public static double MECANUM_WHEEL_DIAMETER = 14.0; // Centimeters

    public static volatile double DRIVE_CURVE = 1.5;
    public static volatile double DRIVE_DEAD_ZONE = 0.05;
    public static volatile double DRIVE_STRAFE_SENSITIVITY = 1;
    public static volatile double DRIVE_FORWARD_SENSITIVITY = 1;
    public static volatile double DRIVE_TURN_SENSITIVITY = 1;

    // Elevator
    public static volatile String SLIDE = "slide_servo";

    // Claw
    public static volatile String CLAW = "claw_servo";

    public static volatile String CLAW_ROTATION = "claw_rotation_servo";

    // Climber
    public static volatile String CLIMBER = "climber";

    // Drone Launcher
    public static volatile String DRONE = "drone_servo";

}
