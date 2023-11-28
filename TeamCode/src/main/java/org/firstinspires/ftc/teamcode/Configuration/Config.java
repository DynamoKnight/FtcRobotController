package org.firstinspires.ftc.teamcode.Configuration;

public class Config {
    //////////////////////
    //VARIABLES
    //////////////////////
    /*
    Robot WIFI: 22556-RC OR 22556-B-RC
    WIFI Password: BHS22556
    IP Address: http://192.168.43.1:8080/
    FTC Setup Slides: https://docs.google.com/presentation/d/17UBroNhJ1QiZ4FxsuMDyzwsqthyI9WFedGaZPvKOX0o/edit?usp=sharing

    Active Configuration:
        Control Hub
            Motor - drive_front_left
            Motor - drive_back_left
            I2C Bus 0 - BHI260APIMU
        Expansion Hub 2
            Motor - drive_front_right
            Motor - drive_back_right
            Motor - climber
            I2C Bus 0 - BNO055IMU
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
    public static volatile String SLIDE = "slide";
    public static volatile String THINGY = "slide_servo";

    // Claw
    public static volatile String CLAW = "claw_servo";

    public static volatile String CLAW_ROTATION = "claw_rotation_servo";

    // Climber
    public static volatile String CLIMBER = "climber";

    // Drone Launcher
    public static volatile String DRONE = "drone_servo";

}
