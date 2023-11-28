package org.firstinspires.ftc.teamcode.MainOpModes;

// The hardware object is referenced

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public OpenCvWebcam camera;
    public CameraPipeline boxLocator;

    //////////////////////
    //CONSTRUCTOR
    //////////////////////
    public NakulAuto(){
        super();
    }

    //////////////////////
    //MAIN METHOD
    //////////////////////
    // This method must be implemented. It is overridden to do the stuff we want.
    @Override
    public void runOpMode() throws InterruptedException {
        double speed = 1;
        int ticks = 0;

        // Initialize Robot and Positions
        robot.init(hardwareMap);
        initCamera(side);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        sleep(2000);
        goToSpike();
        //timer.reset();
        sleep(5000);

        //////////////////////
        // AUTONOMOUS BASED ON POSITION
        //////////////////////
        // Runs the different paths for each starting position
        // BLUE Team Audience Side
        if(side == Side.BLUE_FRONT){
            goToTarget(115, 0.5);
            goToTarget(-8, -0.2);
            sleep(3000);
            turnPID(90);
            //goToTarget(183, 0.8);
        }
        // BLUE Team Wall Side
        else if(side == Side.BLUE_BACK){
            turnPID(90);
        }
        // RED Team Audience Side
        else if(side == Side.RED_FRONT){
            turnPID(90);
        }
        // RED Team Wall Side
        else if(side == Side.RED_BACK){
            turnPID(90);
        }
        //////////////////////
        // DEFAULT AUTONOMOUS
        //////////////////////
        else {
            //goToTarget(115, 0.5);
            //goToTarget(-8, -0.2);
            //sleep(3000);
            //turnPID(90);
        }

    }
    // Takes an enum to determine the starting position
    public void runOpMode(Side side) throws InterruptedException {
        this.side = side;
        runOpMode();
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
        goToTarget(100, 0.5);
        // Sleepy time...
        sleep(3000);

        turnPID(90);
        sleep(2000);
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
    public void goToSpike(){
        CameraPipeline.Position position = boxLocator.getPos();
        // Goes to the target spike mark, then returns to origin
        if(position == CameraPipeline.Position.CENTER){
            telemetry.addData("Object Location", "Center");
            goToTarget(5, 0.5);
            sleep(3000);
            goToTarget(-5, 0.5);
        }
        else if(position == CameraPipeline.Position.LEFT){
            telemetry.addData("Object Location", "LEFT");
            goToTarget(5, 0.5);
            sleep(3000);
            goToTarget(-5, 0.5);
        }
        else if(position == CameraPipeline.Position.RIGHT){
            telemetry.addData("Object Location", "RIGHT");
            goToTarget(5, 0.5);
            sleep(3000);
            goToTarget(-5, 0.5);
        }
        telemetry.update();
    }

    // Moves the robot to the desired target destination
    public void goToTarget(double centimeters, double power){

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

        robot.frontRight.setTargetPosition(robot.distanceToTicks(centimeters));
        robot.frontLeft.setTargetPosition(robot.distanceToTicks(centimeters));
        robot.backRight.setTargetPosition(robot.distanceToTicks(centimeters));
        robot.backLeft.setTargetPosition(robot.distanceToTicks(centimeters));

        robot.setMotorPower(power);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Checks to see if the target destination is reached
        while(opModeIsActive() && robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()){
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

    // Resets the angle of the robot. Does not change the absolute angle, but the relative angle.
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
        while (opModeIsActive() && Math.abs(error) > 2){
            // If the error is in the negative direction, go -0.3 power
            double motorPower = (error < 0 ? -0.3 : 0.3);
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
        while (opModeIsActive() && Math.abs(targetAngle - getAbsoluteAngle()) > 0.5 || pid.getLastSlope() > 0.75) {
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
}

