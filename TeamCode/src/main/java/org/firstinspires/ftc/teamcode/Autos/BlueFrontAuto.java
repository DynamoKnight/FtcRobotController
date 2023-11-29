package org.firstinspires.ftc.teamcode.Autos;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MainOpModes.NakulAuto.Side;

import org.firstinspires.ftc.teamcode.MainOpModes.NakulAuto;

//@Disabled
@Autonomous(name = "BlueFrontAuto", group = "Positions")
//@TeleOp

public class BlueFrontAuto extends LinearOpMode {
    //////////////////////
    //VARIABLES
    //////////////////////
    NakulAuto auto;
    //////////////////////
    //METHODS
    //////////////////////
    @Override
    public void runOpMode() throws InterruptedException {
        auto = new NakulAuto(hardwareMap, telemetry);
        auto.runOpMode(NakulAuto.Side.BLUE_FRONT);
    }
}