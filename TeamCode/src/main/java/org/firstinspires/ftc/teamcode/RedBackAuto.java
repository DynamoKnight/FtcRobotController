package org.firstinspires.ftc.teamcode;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RedBackAuto", group = "")
//@TeleOp

public class RedBackAuto extends LinearOpMode {
    //////////////////////
    //VARIABLES
    //////////////////////
    HardwareBot robot = new HardwareBot();
    private ElapsedTime runtime = new ElapsedTime();

    //////////////////////
    //MAIN METHOD
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
            return;
        }
    }

}

