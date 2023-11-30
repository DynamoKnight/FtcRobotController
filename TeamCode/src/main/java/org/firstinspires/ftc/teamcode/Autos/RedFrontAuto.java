package org.firstinspires.ftc.teamcode.Autos;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MainOpModes.NakulAuto;

//@Disabled
@Autonomous(name = "RedFrontAuto", group = "Positions")
//@TeleOp

public class RedFrontAuto extends LinearOpMode {
    //////////////////////
    //VARIABLES
    //////////////////////
    public NakulAuto auto;
    //////////////////////
    //METHODS
    //////////////////////
    @Override
    public void runOpMode() throws InterruptedException {
        auto = new NakulAuto(hardwareMap, telemetry, NakulAuto.Side.RED_FRONT);
        waitForStart();
        auto.runOpMode();
    }
}