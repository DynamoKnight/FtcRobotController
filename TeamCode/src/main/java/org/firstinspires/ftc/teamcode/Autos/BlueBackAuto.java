package org.firstinspires.ftc.teamcode.Autos;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// Imports the enum so it can be accessed

import org.firstinspires.ftc.teamcode.MainOpModes.NakulAuto;

//@Disabled
@Autonomous(name = "BlueBackAuto", group = "Positions")
//@TeleOp

public class BlueBackAuto extends NakulAuto{
    //////////////////////
    //VARIABLES
    //////////////////////

    //////////////////////
    //CONSTRUCTOR
    //////////////////////
    public BlueBackAuto(){
        super();
    }

    //////////////////////
    //METHODS
    //////////////////////
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(Side.BLUE_BACK);
    }
}