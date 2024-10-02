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
        Webcam 1 - B50A22EO
        Control Hub
            0 Motor - drive_front_left
            1 Motor - drive_back_left
            2 Servo - claw_servo
            I2C Bus 0 BHI260AP IMU - imu2
        Expansion Hub 2
            0 Motor - drive_front_right
            1 Motor - drive_back_right
            2 Motor - climber
            0 Servo - grab_servo_right
            1 Servo - grab_servo
            2 Servo - grab_servo_left
            3 Servo - drone_servo
            I2C Bus 0 BNO055 IMU - imu
     */

    // Drivetrain
    public static volatile String DRIVE_FRONT_LEFT = "drive_front_left";
    public static volatile String DRIVE_FRONT_RIGHT = "drive_front_right";
    public static volatile String DRIVE_BACK_LEFT = "drive_back_left";
    public static volatile String DRIVE_BACK_RIGHT = "drive_back_right";
    public static volatile double ENCODER_RESOLUTION = 537.7;
    // 9.6 cm small, 14.0 cm big
    public static double MECANUM_WHEEL_DIAMETER = 14.0; // Centimeters
    public static double MECANUM_WHEEL_DIAMETER2 = 9.6;

    public static volatile double DRIVE_CURVE = 1.5;
    public static volatile double DRIVE_DEAD_ZONE = 0.05;
    public static volatile double DRIVE_STRAFE_SENSITIVITY = 1;
    public static volatile double DRIVE_FORWARD_SENSITIVITY = 1;
    public static volatile double DRIVE_TURN_SENSITIVITY = 1;

    // Elevator
    public static volatile String SLIDE_LEFT = "slide_left";
    public static volatile String SLIDE_RIGHT = "slide_right";
    public static volatile String AUTO_CLAW = "slide_servo";

    // Claw
    public static volatile String CLAW = "claw_servo";

    public static volatile String CLAW_ROTATION = "claw_rotation_servo";

    // Grabber
    public static volatile String GRABBER = "grabber_servo";
    public static volatile String GRAB_LEFT = "grab_servo_left";
    public static volatile String GRAB_RIGHT = "grab_servo_right";

    // Climber
    public static volatile String CLIMBER = "climber";

    // Drone Launcher
    public static volatile String DRONE = "drone_servo";

}
