package org.firstinspires.ftc.teamcode.MainOpModes;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Configuration.HardwareBot2;

//@Disabled
@TeleOp(name = "TeleOp2")
//@Autonomous

public class NakulTeleOp2 extends LinearOpMode {
    //////////////////////
    //VARIABLES
    //////////////////////
    HardwareBot2 robot = new HardwareBot2();
    NakulAuto auto;

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
            else {
                speed = 0.5;
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
