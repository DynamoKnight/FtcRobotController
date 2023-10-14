package org.firstinspires.ftc.teamcode;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "NakulTeleOp", group = "")
//@Autonomous

public class NakulOpMode extends LinearOpMode {
    //////////////////////
    //VARIABLES
    //////////////////////
    HardwareBot robot = new HardwareBot();

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

            //The range of the joystick is from -1 to 1

            // Mecanum drive is controlled with three axes: drive (front-and-back),
            // strafe (left-and-right), and twist (rotating the whole chassis).
            double drive  = gamepad1.left_stick_y * -1;
            double strafe = gamepad1.left_stick_x;
            double twist  = gamepad1.right_stick_x;

            // You may need to multiply some of these by -1 to invert direction of
            // the motor.  This is not an issue with the calculations themselves.
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
            // Slow mode
            if (gamepad1.left_bumper){
                speed = 0.4;
            }
            else{
                speed = 1;
            }

            if (gamepad1.dpad_up){
                ticks += 50;
                robot.elevator.setTargetPosition(ticks);
                robot.elevator.setPower(0.8);
            }
            if (gamepad1.dpad_down){
                ticks -= 50;
                robot.elevator.setTargetPosition(ticks);
                robot.elevator.setPower(0.4);
            }
            // Rotate 90 degress left
            if (gamepad1.dpad_left){
                robot.frontRight.setPower(0.8);
                robot.backRight.setPower(0.8);
                robot.frontLeft.setPower(-0.8);
                robot.backLeft.setPower(-0.8);
            }
            // Rotate 90 degress right
            if (gamepad1.dpad_right) {

            }

            // Open Claw
            if(gamepad1.a){
                robot.servoClaw.setPosition(0);
            }
            // Close Claw
            if (gamepad1.x){
                robot.servoClaw.setPosition(1);
            }

            telemetry.addData("Status", "Running: Nakul is" + speed + "% EPIC!");
            telemetry.update();

            robot.waitForTick(40);
        }
    }



}
