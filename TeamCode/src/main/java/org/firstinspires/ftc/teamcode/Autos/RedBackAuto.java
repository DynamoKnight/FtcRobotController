package org.firstinspires.ftc.teamcode.Autos;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.NakulAuto.Side;

import org.firstinspires.ftc.teamcode.NakulAuto;

//@Disabled
@Autonomous(name = "RedBackAuto", group = "")
//@TeleOp

public class RedBackAuto extends LinearOpMode {
    //////////////////////
    //VARIABLES
    //////////////////////
    public NakulAuto auto = new NakulAuto();

    //////////////////////
    //METHODS
    //////////////////////
    @Override
    public void runOpMode() throws InterruptedException {
        auto.runOpMode(Side.RED_BACK);
    }
}