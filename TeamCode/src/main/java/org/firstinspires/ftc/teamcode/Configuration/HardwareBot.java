/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Configuration;

import android.transition.Slide;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Configuration.Config;

public class HardwareBot {
    //////////////////////
    //VARIABLES
    //////////////////////

    // Define Motor and Servo objects
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor climber;
    public Servo claw;
    public Servo drone;
    public BNO055IMU imu;
    public Orientation angles;
    public double encoder_resolution;
    public double mc_diameter;
    public DcMotor slide;
    public Servo thingy;

    public ElapsedTime period = new ElapsedTime();

    //////////////////////
    //CONSTRUCTOR
    //////////////////////
    public HardwareBot() {}

    //////////////////////
    //METHODS
    //////////////////////
    public void init(HardwareMap hardwareMap) throws InterruptedException {

        // Define and Initialize Motors
        // Names need to exactly match with those in the control hub
        frontLeft = hardwareMap.get(DcMotor.class, Config.DRIVE_FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, Config.DRIVE_FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, Config.DRIVE_BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, Config.DRIVE_BACK_RIGHT);
        climber = hardwareMap.get(DcMotor.class, Config.CLIMBER);
        claw = hardwareMap.get(Servo.class, Config.CLAW);
        slide = hardwareMap.get(DcMotor.class, Config.SLIDE);
        thingy = hardwareMap.get(Servo.class, Config.THINGY);

        // The Left side axle points are in opposite direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        setMotorPower(0);

        // The drive motors don't use motors
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // It brakes when at 0 power
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initializing the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO55IMUCalibration.json";
        // Allows the values to be tracked
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        // Indicates the algorithm used to track the velocity and position
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //RevHubOrientationOnRobot.LogoFacingDirection.RIGHT
        //RevHubOrientationOnRobot.UsbFacingDirection.UP

        // Slide Motor
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        encoder_resolution = Config.ENCODER_RESOLUTION;
        mc_diameter = Config.MECANUM_WHEEL_DIAMETER;
    }

    public void waitForTick(long periodMs) throws InterruptedException {

        long remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0){
            Thread.sleep(remaining);
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    // Converts the target distance to ticks
    public int distanceToTicks(double centimeters){
        // Circumference = Diameter * pi
        // Revolutions = Distance / Circumference
        // Number of Ticks = Encoder * Revolutions
        double revolutions = centimeters / (mc_diameter * Math.PI);
        int ticks = (int)(encoder_resolution * revolutions);
        return ticks;
    }

    // Sets the same power to all motors
    public void setMotorPower(double power){
        setMotorPower(power, power, power, power);
    }

    // Individually sets the power of the motors
    public void setMotorPower(double fl, double fr, double bl, double br){
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }



}
