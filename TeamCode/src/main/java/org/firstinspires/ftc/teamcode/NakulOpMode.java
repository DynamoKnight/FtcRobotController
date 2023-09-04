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
    public void runOpMode() {
        double speed = 1;
        double leftPower;
        double rightPower;

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //timer.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Allows robot movement

            // Slow mode
            if (gamepad1.left_bumper){
                speed = 0.4;
            }
            else{
                speed = 1;
            }

            //The range of the joystick is from 0-1
            leftPower = -this.gamepad1.left_stick_y;
            rightPower = this.gamepad1.right_stick_y;
            robot.frontLeft.setPower(leftPower);
            robot.backLeft.setPower(leftPower);
            robot.frontRight.setPower(rightPower);
            robot.backRight.setPower(rightPower);

            telemetry.addData("Status", "Running: Nakul is" + rightPower + "% cool!");
            telemetry.update();

            //robot.waitForTick(40);
        }
    }

}
