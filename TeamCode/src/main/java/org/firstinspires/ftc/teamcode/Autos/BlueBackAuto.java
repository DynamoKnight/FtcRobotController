package org.firstinspires.ftc.teamcode.Autos;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// Imports the enum so it can be accessed

import org.firstinspires.ftc.teamcode.MainOpModes.NakulAuto;

//@Disabled
@Autonomous(name = "BlueBackAuto", group = "Positions")
//@TeleOp

public class BlueBackAuto extends LinearOpMode {
    //////////////////////
    //VARIABLES
    //////////////////////
    public NakulAuto auto;
    //////////////////////
    //METHODS
    //////////////////////
    @Override
    public void runOpMode() throws InterruptedException {
        //auto = new NakulAuto(new BlueBackAuto());
        auto = new NakulAuto(hardwareMap, telemetry, NakulAuto.Side.BLUE_BACK);
        waitForStart();
        auto.runOpMode();
    }
}