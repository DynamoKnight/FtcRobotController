package org.firstinspires.ftc.teamcode.MainOpModes;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Configuration.HardwareBot;

//@Disabled
@TeleOp(name = "TeleOp")
//@Autonomous

public class NakulTeleOp extends LinearOpMode {
    //////////////////////
    //VARIABLES
    //////////////////////
    HardwareBot robot = new HardwareBot();
    NakulAuto auto;

    //////////////////////
    //METHODS
    //////////////////////
    // This method must be implemented. It is overridden to do the stuff we want.
    @Override
    public void runOpMode() throws InterruptedException {
        double speed = 1;
        int ticks = 0;
        // Grabber toggle variables
        double[] grab_pos = {0.525, 0.4, 0.25};
        int cur_idx = 0;
        int dir = 1;
        boolean grabIsHeld = false;
        boolean lGrabIsHeld = false;
        boolean rGrabIsHeld = false;

        robot.init(hardwareMap);

        // 1500 is 0.5, 2200 is 1
        robot.grabber.setPosition(0.525);
        robot.grab_right.setPosition(0.4);
        robot.grab_left.setPosition(0.45);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //timer.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //////////////////////
            //GAMEPAD 1
            //////////////////////
            // Prevents stick drift
            if (gamepad1.right_stick_x < 0.1 && gamepad1.right_stick_x > -0.1){
                gamepad1.right_stick_x = 0;
            }
            if (gamepad1.left_stick_y < 0.1 && gamepad1.left_stick_y > -0.1){
                gamepad1.left_stick_y = 0;
            }
            if (gamepad1.left_stick_x < 0.1 && gamepad1.left_stick_x > -0.1){
                gamepad1.left_stick_x = 0;
            }

            // Moves robot
            drivetrain(speed);

            // Sprint button
            if (gamepad1.left_trigger > 0.5){
                speed = 0.8;
            }
            // Regular Speed
            else{
                speed = 0.5;
            }

            // Climber moves up
            if (gamepad1.dpad_up) {
                if (robot.climber != null) {
                    robot.climber.setPower(1);
                }
            }
            // Climber moves down
            else if (gamepad1.dpad_down) {
                if (robot.climber != null) {
                    robot.climber.setPower(-1);
                }
            }
            // Climber stops
            else {
                if (robot.climber != null) {
                    robot.climber.setPower(0);
                }
            }

            // When Start + A is pressed, this runs
            /*if (gamepad1.a) {
            }*/

            // Toggle Grabber position
            // isHeld ensures that the position doesn't change
            // while the button is being held
            if (gamepad1.y & !grabIsHeld) {
                if (robot.grabber != null) {
                    robot.grabber.setPosition(grab_pos[cur_idx]);
                    if ((cur_idx + dir) > grab_pos.length - 1){
                        dir = -1;
                    }
                    else if ((cur_idx + dir) < 0){
                        dir = 1;
                    }
                    cur_idx += dir;
                }
                grabIsHeld = true;
            }
            // Indicates that it isn't held
            if (!gamepad1.y){
                grabIsHeld = false;
            }
            // Toggles Left Grabber
            if (gamepad1.left_bumper & !lGrabIsHeld){
                if (robot.grab_left != null){
                    if ((robot.grab_left.getPosition() == 0.35)) {
                        // Open
                        robot.grab_left.setPosition(0.55);
                    } else {
                        // Close
                        robot.grab_left.setPosition(0.35);
                    }
                }
                lGrabIsHeld = true;
            }
            if (!gamepad1.left_bumper){
                lGrabIsHeld = false;
            }
            // Toggles Right Grabber
            if (gamepad1.right_bumper & !rGrabIsHeld){
                if (robot.grab_right != null){
                    if ((robot.grab_right.getPosition() == 0.68)) {
                        // Open
                        robot.grab_right.setPosition(0.52);
                    } else {
                        // Close
                        robot.grab_right.setPosition(0.68);
                    }
                }
                rGrabIsHeld = true;
            }
            if (!gamepad1.right_bumper){
                rGrabIsHeld = false;
            }

            // Launch Drone
            if(gamepad1.right_trigger > 0.5){
                if (robot.drone != null){
                    robot.drone.setPosition(1);
                }
            }

            //////////////////////
            //GAMEPAD 2
            //////////////////////
            // Nothing

            telemetry.addData("Status", "Running: Nakul is" + speed + "% EPIC!");
            telemetry.update();

            robot.waitForTick(40);
        }
    }

    // Sets the power of the individual motors to move, strafe, and rotate
    public void drivetrain(double speed){
        //The range of the joystick is from -1 to 1

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = gamepad1.left_stick_y * -1;
        double strafe = gamepad1.left_stick_x;
        double twist  = gamepad1.right_stick_x;

        // You may need to multiply some of these by -1 to invert direction of
        // the motor. This is not an issue with the calculations themselves.
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        // The motors only take values between [-1,1]
        // When a robot turns,
        // the power of a wheel may exceed the magnitude of 1.
        // In order to fix this, that wheel must be set to 1
        // and every other wheel’s power must normalize to 1(same ratio).

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for (double v : speeds) {
            if (Math.abs(v) > max){
                max = Math.abs(v);
            }
        }

        // If and only if the maximum is outside of the range we want it to be(1),
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            // The max speed becomes 1
        }

        // Apply the calculated values to the motors.
        robot.frontLeft.setPower(speeds[0] * speed);
        robot.frontRight.setPower(speeds[1] * speed);
        robot.backLeft.setPower(speeds[2] * speed);
        robot.backRight.setPower(speeds[3] * speed);
    }

}
