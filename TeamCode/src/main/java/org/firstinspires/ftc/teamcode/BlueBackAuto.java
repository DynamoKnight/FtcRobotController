package org.firstinspires.ftc.teamcode;

// The hardware object is referenced
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "BlueBackAuto", group = "")
//@TeleOp

public class BlueBackAuto extends NakulAuto {


    //////////////////////
    //METHODS
    //////////////////////
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(Side.BLUE_BACK);
    }
}