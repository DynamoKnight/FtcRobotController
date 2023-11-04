package org.firstinspires.ftc.teamcode;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "NakulTeleOp", group = "")
//@Autonomous

public class NakulOpMode extends LinearOpMode {
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

        robot.init(hardwareMap);
        auto = new NakulAuto();


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

            // Rotate 90 degress left
            if (gamepad1.dpad_left){
                //auto.turnPID(90);
            }
            // Rotate 90 degress right
            if (gamepad1.dpad_right) {
                //auto.turnPID(-90);
            }
            // Moves Linear slide up
            if (gamepad1.a){
                if (robot.elevator != null){
                    robot.elevator.setPosition(0.45);
                    /*
                    ticks += 50;
                    robot.elevator.setTargetPosition(ticks);
                    robot.elevator.setPower(0.8);
                    */
                }

            }
            // Moves Linear slide down
            if (gamepad1.b) {
                if (robot.elevator != null) {
                    robot.elevator.setPosition(0);
                    /*
                    ticks -= 50;
                    robot.elevator.setTargetPosition(ticks);
                    robot.elevator.setPower(0.4);
                    */
                }
            }

            // Moves Climber Up
            if(gamepad1.dpad_up) {
                if (robot.climber != null) {
                    robot.climber.setPower(0.5);
                }
            }
            // Climber hooks on
            if(gamepad1.dpad_down) {
                if (robot.climber != null) {
                    robot.climber.setPower(-0.5);
                }
            }


            // Launch Drone
            if(gamepad1.right_trigger > 0.1){
                if (robot.drone != null){
                    robot.drone.setPosition(0);
                }
            }
            // Open Claw
            if (gamepad1.x){
                if (robot.claw != null){
                    robot.claw.setPosition(0.65);
                }
            }
            // Close Claw
            if (gamepad1.y){
                if (robot.claw != null){
                    robot.claw.setPosition(0.55);
                }
            }

            telemetry.addData("Status", "Running: Nakul is" + speed + "% EPIC!");
            telemetry.update();

            robot.waitForTick(40);
        }
    }



}
