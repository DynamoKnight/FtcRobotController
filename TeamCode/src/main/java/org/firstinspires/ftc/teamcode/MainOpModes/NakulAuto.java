package org.firstinspires.ftc.teamcode.MainOpModes;

// The hardware object is referenced

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Configuration.HardwareBot;
import org.firstinspires.ftc.teamcode.Test.TestPIDController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.function.BooleanSupplier;
@Disabled
@Autonomous(name = "NakulAuto")

public class NakulAuto extends LinearOpMode{
    //////////////////////
    //VARIABLES
    //////////////////////
    // An enum is a group of constants.
    public enum Side{
        BLUE_FRONT,
        BLUE_BACK,
        RED_FRONT,
        RED_BACK
    }

    public HardwareBot robot = new HardwareBot();
    public ElapsedTime runtime;

    public Orientation previousAngles = new Orientation();
    public double currentAngle = 0.0;
    public Side side;
    public double speed = 0.25;

    public OpenCvWebcam camera;
    public CameraPipeline boxLocator;

    public LinearOpMode auto;

    //////////////////////
    //CONSTRUCTOR
    //////////////////////
    public NakulAuto(){}

    public NakulAuto(LinearOpMode auto, Side side){
        // auto is a specific reference to "this"
        this.auto = auto;
        this.hardwareMap = auto.hardwareMap;
        this.telemetry = auto.telemetry;
        this.side = side;
    }

    //////////////////////
    //MAIN METHOD
    //////////////////////
    // This method must be implemented. It is overridden to do the stuff we want.
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Robot and Positions
        initCamera(this.side);
        robot.init(hardwareMap);
        robot.grab_left.setPosition(0.35);
        robot.grab_right.setPosition(0.68);

        this.telemetry.addData("Status", "Initialized at " + side);
        this.telemetry.update();

        auto.waitForStart();
        robot.grabber.setPosition(0.525);
        sleep(500);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        // Gets the current position of the object
        CameraPipeline.Position pos = boxLocator.getPos();
        // Ends the camera stream
        camera.closeCameraDeviceAsync(() -> {
            telemetry.addData("Object Position", pos);
            telemetry.update();
        });

        goToSpike(pos);

        telemetry.addData("Status", "Spike Reached");
        telemetry.update();
        //timer.reset();
        sleep(2000);

        //////////////////////
        // AUTONOMOUS BASED ON POSITION
        //////////////////////
        // Runs the different paths for each starting position
        // BLUE Team Audience Side
        if(side == Side.BLUE_FRONT){

        }
        // BLUE Team Wall Side
        else if(side == Side.BLUE_BACK){

        }
        // RED Team Audience Side
        else if(side == Side.RED_FRONT){

        }
        // RED Team Wall Side
        else if(side == Side.RED_BACK){

        }

        telemetry.addData("Status", "Finished Auto");
        telemetry.update();

    }

    //////////////////////
    //METHODS
    //////////////////////

    // Creates and starts the camera
    public void initCamera(Side side){
        // Gets the camera object
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        boxLocator = new CameraPipeline(telemetry, side);

        camera.setPipeline(boxLocator);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            // Sets the Camera Resolution and Orientation
            public void onOpened() {
                camera.startStreaming(320, 240 , OpenCvCameraRotation.UPRIGHT);
            }
            // Reports error
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();

            }
        });
        // Displays on FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(camera, 0);
    }

    // Gets the position of the object on the spike
    // and places the pixel onto that spike
    public void goToSpike(CameraPipeline.Position position){
        telemetry.addData("Position", position);
        // Goes to the target spike mark, then returns to origin

        // CENTER SPIKE MARK
        if(position == CameraPipeline.Position.CENTER) {
            telemetry.addData("Object Location", "Center");

            if (side == Side.BLUE_BACK) {
                goToTarget(56, false);
                sleep(1000);
                robot.grab_left.setPosition(0.55);
                goToTarget(-10, false);
                turnTo(85);
                robot.grabber.setPosition(0.4);
                robot.climber.setPower(1);
                sleep(1000);
                robot.climber.setPower(0);
                sleep(500);
                goToTarget(40, false);
                sleep(500);
                goToTarget(30, true);
                sleep(500);
                goToTarget(42.5, false);
                sleep(1500);
                robot.grab_right.setPosition(0.55);
                sleep(150);
                goToTarget(-10, false);
                goToTarget(-70, true);
                goToTarget(25, false);
            }
            else if (side == Side.BLUE_FRONT) {
                goToTarget(10, true);
                goToTarget(55, false);
                sleep(750);
                robot.grab_left.setPosition(0.55);
                sleep(500);
                robot.climber.setPower(1);
                sleep(1000);
                robot.climber.setPower(0);
                goToTarget(70, false);
                turnTo(85);
                goToTarget(150, false);
                robot.grabber.setPosition(0.4);
                goToTarget(-67, true);
                goToTarget(56, false);
                sleep(1500);
                robot.grab_right.setPosition(0.55);
                sleep(250);
                goToTarget(-10, false);
                goToTarget(59, true);
                goToTarget(27, false);
            }
            else if (side == Side.RED_BACK) {
                goToTarget(56, false);
                sleep(1000);
                robot.grab_left.setPosition(0.55);
                goToTarget(-10, false);
                turnTo(-85);
                robot.grabber.setPosition(0.4);
                robot.climber.setPower(1);
                sleep(1000);
                robot.climber.setPower(0);
                sleep(500);
                goToTarget(40, false);
                sleep(500);
                goToTarget(-30, true);
                sleep(500);
                goToTarget(42.5, false);
                sleep(1500);
                robot.grab_right.setPosition(0.55);
                sleep(150);
                goToTarget(-10, false);
                goToTarget(70, true);
                goToTarget(25, false);
            }
            else if (side == Side.RED_FRONT){
                goToTarget(-10, true);
                goToTarget(55, false);
                sleep(750);
                robot.grab_left.setPosition(0.55);
                sleep(500);
                robot.climber.setPower(1);
                sleep(1000);
                robot.climber.setPower(0);
                goToTarget(70, false);
                turnTo(-85);
                goToTarget(150, false);
                robot.grabber.setPosition(0.4);
                goToTarget(67, true);
                goToTarget(56, false);
                sleep(1500);
                robot.grab_right.setPosition(0.55);
                sleep(250);
                goToTarget(-10, false);
                goToTarget(-59, true);
                goToTarget(27, false);
            }
        }
        // LEFT SPIKE MARK
        else if(position == CameraPipeline.Position.LEFT){
            telemetry.addData("Object Location", "LEFT");

            if (side == Side.BLUE_BACK){
                goToTarget(22, speed, true);
                sleep(500);
                goToTarget(30, speed, false);
                sleep(1000);
                robot.grab_left.setPosition(0.55);
                goToTarget(-10, speed, false);
                turnTo(85);
                robot.climber.setPower(1);
                sleep(1000);
                robot.climber.setPower(0);
                robot.grabber.setPosition(0.4);
                goToTarget(40, speed, false);
                goToTarget(35, speed, true);
                goToTarget(15, speed, false);
                sleep(1500);
                robot.grab_right.setPosition(0.55);
                sleep(250);
                goToTarget(-10, speed, false);
                goToTarget(-45, speed, true);
                goToTarget(25, speed, false);
            }
            else if (side == Side.BLUE_FRONT) {
                goToTarget(29, speed, true);
                sleep(1000);
                goToTarget(40, speed, false);
                robot.grab_left.setPosition(0.55);
                sleep(500);
                robot.climber.setPower(1);
                sleep(1000);
                robot.climber.setPower(0);
                goToTarget(75, speed,false);
                turnTo(85);
                goToTarget(203, speed,false);
                robot.grabber.setPosition(0.4);
                goToTarget(-35, speed,true);
                goToTarget(35, speed,false);
                sleep(1500);
                robot.grab_right.setPosition(0.55);
                sleep(250);
                goToTarget(-10, speed, false);
                goToTarget(35, speed,true);
                goToTarget(20, speed,false);
            }
            else if (side == Side.RED_BACK){
                goToTarget(40, false);
                turnTo(40);
                goToTarget(10, false);
                sleep(500);
                robot.grab_left.setPosition(0.55);
                sleep(250);
                goToTarget(-10, false);
                turnTo(-87.5);
                robot.climber.setPower(1);
                sleep(1000);
                robot.climber.setPower(0);
                robot.grabber.setPosition(0.4);
                goToTarget(50, false);
                goToTarget(-45, true);
                goToTarget(27.5, false);
                sleep(1000);
                robot.grab_right.setPosition(0.55);
                sleep(250);
                goToTarget(-10, false);
                goToTarget(80, true);
                goToTarget(15, false);

            }
            else if (side == Side.RED_FRONT) {
                goToTarget(-30, speed, true);
                sleep(1000);
                goToTarget(40, speed, false);
                robot.grab_left.setPosition(0.55);
                sleep(500);
                robot.climber.setPower(1);
                sleep(1000);
                robot.climber.setPower(0);
                goToTarget(75, speed,false);
                turnTo(-85);
                goToTarget(203, speed,false);
                robot.grabber.setPosition(0.4);
                goToTarget(37.5, speed,true);
                goToTarget(32.5, speed,false);
                sleep(1500);
                robot.grab_right.setPosition(0.55);
                sleep(250);
                goToTarget(-10, speed, false);
                goToTarget(-30, speed,true);
                goToTarget(20, speed,false);
            }

        }
        // RIGHT SPIKE MARK
        else if(position == CameraPipeline.Position.RIGHT){
            telemetry.addData("Object Location", "RIGHT");

            if (side == Side.BLUE_BACK){
                goToTarget(40, false);
                turnTo(-40);
                goToTarget(10, false);
                sleep(500);
                robot.grab_left.setPosition(0.55);
                sleep(250);
                goToTarget(-10, false);
                turnTo(87.5);
                robot.climber.setPower(1);
                sleep(1000);
                robot.climber.setPower(0);
                robot.grabber.setPosition(0.4);
                goToTarget(50, false);
                goToTarget(45, true);
                goToTarget(27.5, false);
                sleep(1000);
                robot.grab_right.setPosition(0.55);
                sleep(250);
                goToTarget(-10, false);
                goToTarget(-80, true);
                goToTarget(15, false);
            }
            else if (side == Side.BLUE_FRONT) {
                goToTarget(22, speed, true);
                sleep(1000);
                goToTarget(33, speed, false);
                sleep(1000);
                robot.grab_left.setPosition(0.55);
                sleep(500);
                robot.climber.setPower(1);
                sleep(1000);
                robot.climber.setPower(0);
                goToTarget(40, speed,false);
                turnTo(85);
                goToTarget(150, speed,false);
                robot.grabber.setPosition(0.4);
                goToTarget(-50, speed,true);
                goToTarget(45, speed,false);
                robot.grab_right.setPosition(0.55);
                goToTarget(-10, speed,false);
                goToTarget(60, speed,true);
                goToTarget(25, speed,false);

            }
            else if (side == Side.RED_BACK){
                goToTarget(22, speed, true);
                sleep(500);
                goToTarget(30, speed, false);
                sleep(1000);
                robot.grab_left.setPosition(0.55);
                sleep(250);
                goToTarget(-10, speed, false);
                turnTo(-85);
                robot.climber.setPower(1);
                sleep(1000);
                robot.climber.setPower(0);
                robot.grabber.setPosition(0.4);
                goToTarget(40, speed, false);
                goToTarget(-32.5, speed, true);
                goToTarget(15, speed, false);
                sleep(1500);
                robot.grab_right.setPosition(0.55);
                sleep(250);
                goToTarget(-10, speed, false);
                goToTarget(47.5, speed, true);
                goToTarget(25, speed, false);
            }
            else if (side == Side.RED_FRONT) {
                goToTarget(-10, true);
                goToTarget(40, false);
                turnTo(-40);
                goToTarget(10, false);
                sleep(250);
                robot.grab_left.setPosition(0.55);
                sleep(250);
                goToTarget(-10, false);
                sleep(1000);
                turnTo(5);

                goToTarget(80, false);
                robot.climber.setPower(1);
                sleep(1000);
                robot.climber.setPower(0);
                robot.grabber.setPosition(0.4);
                turnTo(-85);
                goToTarget(150, false);
                goToTarget(80, true);
                goToTarget(62, false);
                sleep(1000);
                robot.grab_right.setPosition(0.55);
                sleep(250);
                goToTarget(-10, false);
                goToTarget(-70, true);
                goToTarget(20, false);
            }

        }
        telemetry.update();
    }

    /**
     * Different signature with default power value
     * @param centimeters is the displacement needed to travel.
     * @param strafe is the desired speed.
     */
    public void goToTarget(double centimeters, boolean strafe){
        goToTarget(centimeters, speed, strafe);
    }

    /**
     * Moves the robot to the desired target destination
     * @param centimeters is the displacement needed to travel.
     * @param power is the desired speed.
     * @param strafe indicates whether the robot is moving longitudinal or lateral
    **/
    public void goToTarget(double centimeters, double power, boolean strafe){
        // Allows ticks and position to be tracked
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // The target position in units of ticks
        double targetTicks = robot.frontLeft.getCurrentPosition() + robot.distanceToTicks(centimeters);

        // Allows strafe
        if (strafe) {
            robot.frontRight.setTargetPosition(-robot.distanceToTicks(centimeters));
            robot.frontLeft.setTargetPosition(robot.distanceToTicks(centimeters));
            robot.backRight.setTargetPosition(robot.distanceToTicks(centimeters));
            robot.backLeft.setTargetPosition(-robot.distanceToTicks(centimeters));
        }
        else{
            robot.frontRight.setTargetPosition(robot.distanceToTicks(centimeters));
            robot.frontLeft.setTargetPosition(robot.distanceToTicks(centimeters));
            robot.backRight.setTargetPosition(robot.distanceToTicks(centimeters));
            robot.backLeft.setTargetPosition(robot.distanceToTicks(centimeters));
        }

        robot.setMotorPower(power);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Checks to see if the target destination is reached
        while(auto.opModeIsActive() && robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()){
            telemetry.addData("Front_Left Position: ", robot.frontLeft.getCurrentPosition());
            telemetry.update();
        }
        // Stops the robot when it leaves the loop
        robot.setMotorPower(0);

        // Resets robot to work without encoders
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Resets the angle of the robot. Does not change the absolute angle, but the relative angle.
     */
    public void resetAngle(){
        previousAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentAngle = 0.0;
    }

    // Returns the current Angle of the robot
    public double getAngle(){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // The change in angle from the previous to the current
        double deltaAngle = orientation.firstAngle - previousAngles.firstAngle;
        // Normalizes the angle so that it is between -180 to 180
        if (deltaAngle > 180){
            deltaAngle -= 360;
        }
        else if (deltaAngle < -180){
            deltaAngle += 360;
        }

        currentAngle += deltaAngle;
        previousAngles = orientation;
        telemetry.addData("Heading", orientation.firstAngle);
        //telemetry.addData("Roll", angles.secondAngle);
        //telemetry.addData("Pitch", angles.thirdAngle);
        return currentAngle;
    }

    // Turns the Robot and checks if it has reached it's target
    // RELATIVE TURN: Turns degrees relative to where it is
    public void turn(double degrees){
        resetAngle();
        // How far away the robot is from the target angle
        double error = degrees;
        // Keeps turning until within +- 2 degrees of target
        while (auto.opModeIsActive() && Math.abs(error) > 2){
            // If the error is in the negative direction, go -0.3 power
            double motorPower = (error < 0 ? -0.25 : 0.25);
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            // Updates the error
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }

        robot.setMotorPower(0);
    }

    // Turns the Robot to a target angle
    // ABSOLUTE TURN: Turns degrees based on the initial heading
    public void turnTo(double degrees){
        // Gets the current orientation
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Finds how far it's initial angle is to it's desired angle
        double error = degrees - orientation.firstAngle;
        // Normalizes the error so that is is between -180 to 180
        if (error > 180){
            error -= 360;
        }
        else if (error < -180){
            error += 360;
        }
        // Turns to the absolute angle
        turn(error);
    }

    // Returns the angle of the robot relative to the origin
    public double getAbsoluteAngle(){
        return robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

    // Turns to the absolute angle using PID coefficients
    void turnToPID(double targetAngle){
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        // 50 milliseconds for each update
        telemetry.setMsTransmissionInterval(50);
        // Keeps turning until the robot is within ±1 degree of the target
        while (auto.opModeIsActive() && Math.abs(targetAngle - getAbsoluteAngle()) > 0.5 || pid.getLastSlope() > 0.75) {
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);

            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        // Stops the robot when done
        robot.setMotorPower(0);
    }

    // Turns to the relative angle
    void turnPID(double degrees){
        turnToPID(degrees + getAbsoluteAngle());
    }

    // Returns best angle
    public double angleWrap(double degrees) {
        // Gets angle within ±180
        while (degrees > 180){
            degrees -= 360;
        }
        while (degrees < -180){
            degrees += 360;
        }
        return degrees;
    }

    //////////////////////
    //TESTING METHODS
    //////////////////////

    // Tests the robot with PID
    public void movePIDTest(){
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double targetAngle = 90;
        double targetDistance = robot.distanceToTicks(20);
        TestPIDController test = new TestPIDController(0.01, 0, 0.003);

        double error = targetDistance - robot.frontLeft.getCurrentPosition();
        //double error = angleWrap(targetAngle - robot.imu.getAngularOrientation().firstAngle);

        // Stop when error is within 2 cm or 2 deg
        while (opModeIsActive() && Math.abs(error) > 2){
            double power = test.PIDControl(error);
            //double power = test.PIDControl(error);
            robot.setMotorPower(power);
        }
        robot.setMotorPower(0);


    }

    // Tests the IMU and PID Control System
    public void gyroTest(){
        // Sets the target for the robot to travel
        goToTarget(100, 0.5, false);
        // Sleepy time...
        sleep(3000);

        turnPID(90);
        sleep(2000);
    }
}

