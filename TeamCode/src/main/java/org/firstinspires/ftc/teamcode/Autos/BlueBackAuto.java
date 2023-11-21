package org.firstinspires.ftc.teamcode.Autos;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
// Imports the enum so it can be accessed
import org.firstinspires.ftc.teamcode.NakulAuto.Side;

import org.firstinspires.ftc.teamcode.NakulAuto;

//@Disabled
@Autonomous(name = "BlueBackAuto", group = "")
//@TeleOp

public class BlueBackAuto extends LinearOpMode {
    //////////////////////
    //VARIABLES
    //////////////////////
    public NakulAuto auto = new NakulAuto();

    //////////////////////
    //METHODS
    //////////////////////
    @Override
    public void runOpMode() throws InterruptedException {
        auto.runOpMode(Side.BLUE_BACK);
    }
}