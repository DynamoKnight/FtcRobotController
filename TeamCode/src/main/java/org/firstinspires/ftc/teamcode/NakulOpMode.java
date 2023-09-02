package org.firstinspires.ftc.teamcode;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
//@Autonomous

public class NakulOpMode extends LinearOpMode {

    // This method must be implemented. It is overridden to do the stuff we want.
    @Override
    public void runOpMode() {
        //Names need to exactly match with those in the control hub
        IMU imu = hardwareMap.get(IMU.class, "imu");

        telemetry.addData("Status", "Initialized");
        telemetry.update(); // Basically like a commit in the terminal
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        int count = 0;
        double tgtPower = 0;
        while (opModeIsActive()) {
            // Allows left stick movement
            tgtPower = -this.gamepad1.left_stick_y;
            //motorTest.setPower(tgtPower);
            telemetry.addData("Target Power", tgtPower);
            //telemetry.addData("Motor Power", motorTest.getPower()); // The same as previous

            telemetry.addData("Status", "Running: Nakul is" + count + "% cool!");
            telemetry.update();
            count++;
        }
    }

}
