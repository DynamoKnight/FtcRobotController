package org.firstinspires.ftc.teamcode.Autos;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MainOpModes.NakulAuto.Side;

import org.firstinspires.ftc.teamcode.MainOpModes.NakulAuto;

//@Disabled
@Autonomous(name = "BlueFrontAuto", group = "")
//@TeleOp

public class BlueFrontAuto extends NakulAuto {
    //////////////////////
    //VARIABLES
    //////////////////////

    //////////////////////
    //CONSTRUCTOR
    //////////////////////
    public BlueFrontAuto(){
        super();
    }

    //////////////////////
    //METHODS
    //////////////////////
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(Side.BLUE_FRONT);
    }
}