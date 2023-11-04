package org.firstinspires.ftc.teamcode;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "RedBackAuto", group = "")
//@TeleOp

public class RedBackAuto extends NakulAuto {


    //////////////////////
    //METHODS
    //////////////////////
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(Side.RED_BACK);
    }
}