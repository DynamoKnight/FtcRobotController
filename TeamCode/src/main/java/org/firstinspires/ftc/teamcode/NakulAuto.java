package org.firstinspires.ftc.teamcode;

// The hardware object is referenced
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.annotation.Target;

@Autonomous(name = "NakulAuto", group = "")
//@TeleOp

public class NakulAuto extends LinearOpMode {
    //////////////////////
    //VARIABLES
    //////////////////////
    HardwareBot robot = new HardwareBot();
    private ElapsedTime runtime = new ElapsedTime();

    private Orientation previousAngles = new Orientation();
    private double currentAngle = 0.0;
    //////////////////////
    //METHODS
    //////////////////////
    // This method must be implemented. It is overridden to do the stuff we want.
    @Override
    public void runOpMode() throws InterruptedException {
        double speed = 1;
        int ticks = 0;

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //timer.reset();

        // Run and then Stop after finishing
        // move forward for two seconds
        if (opModeIsActive()) {
            gyroTest();
        }
    }

    // Tests the robot moving forward
    public void moveForward(){

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.frontRight.setTargetPosition(robot.move(20));
        robot.frontLeft.setTargetPosition(robot.move(20));

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(1000);
    }

    // Tests the IMU and PID Control System
    public void gyroTest(){
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(0.5);
        sleep(2000);     // wait for two seconds
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);

        turn(90);
        sleep(3000);
        turnTo(-90);
        sleep(3000);

        turnPID(180);
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
        else if (deltaAngle <= -180){
            deltaAngle += 360;
        }

        currentAngle = deltaAngle;
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
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            // Updates the error
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        robot.setMotorPower(0.0);
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
        else if (error <= -180){
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
        // Keeps turning until the robot is within ±1 degree of the target
        while(opModeIsActive() && Math.abs(targetAngle - getAbsoluteAngle()) > 1){
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
        }
        // Stops the robot when done
        robot.setMotorPower(0);
    }

    // Turns to the relative angle
    void turnPID(double degrees){
        turnToPID(degrees + getAbsoluteAngle());
    }
}

