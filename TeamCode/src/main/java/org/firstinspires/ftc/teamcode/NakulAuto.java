package org.firstinspires.ftc.teamcode;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "NakulAuto", group = "")
//@TeleOp

public class NakulAuto extends LinearOpMode {
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
        /*while (opModeIsActive()) {
            moveForward();


            telemetry.addData("Status", "Running: Nakul is" + speed + "% ROBOT!");
            telemetry.update();

            robot.waitForTick(40);
        }*/

        // move forward for two seconds
        if (opModeIsActive()) {
            robot.frontLeft.setPower(0.5);
            robot.frontRight.setPower(0.5);
            sleep(2000);     // wait for two seconds
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
        }
    }

    public void moveForward(){
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.frontRight.setTargetPosition(robot.spin(5));
        robot.frontLeft.setTargetPosition(robot.spin(5));
        robot.frontRight.setTargetPosition(robot.spin(5));
        robot.frontRight.setTargetPosition(robot.spin(5));

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(1000);
    }


}

