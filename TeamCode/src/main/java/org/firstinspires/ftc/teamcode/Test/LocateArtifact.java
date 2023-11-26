package org.firstinspires.ftc.teamcode.Test;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class LocateArtifact extends OpenCvPipeline {

    //////////////////////////////
    // Properties
    //////////////////////////////
    public static enum Pos{LEFT, CENTER, RIGHT};
    public static final Rect B1 = new Rect(30,120,30,30);
    public static final Rect B2 = new Rect(145,120,30,30);
    public static final Rect B3 = new Rect(260,120,30,30);

    public static final int LOWER_R = 110;
    public static final int LOWER_G = 0;
    public static final int LOWER_B = 0;
    public static final int UPPER_R = 255;
    public static final int UPPER_G = 90;
    public static final int UPPER_B = 255;
    
    // Used for Configuration in FTC Dashboard
    public Telemetry telemetry;
    
    public Pos pos;
    public Mat color;
    
    //////////////////////
    //CONSTRUCTOR
    //////////////////////
    public LocateArtifact(Telemetry telemetry){
        this.telemetry = telemetry;
        this.pos = Pos.CENTER;
        this.color = new Mat();
    }

    


    //////////////////////
    //METHODS
    //////////////////////

    // Shows what the camera is seeing
    @Override
    public Mat processFrame(Mat input){
        Mat tempMat = input;
        // Based on the color channel, it evaluates a greyscale channel where
        // the darkest color is the least of the color, and
        // the brightest color is the most of the color.

        // Converts to BGR because RGB doesn't work for some reason!!!
        Imgproc.cvtColor(input, tempMat, Imgproc.COLOR_RGB2BGR);
        // Creates a small blur so that noise is removed
        Imgproc.GaussianBlur(tempMat, tempMat, new Size(5, 5), 0);
        // Sets bounds for what the channel is based on
        Core.inRange(tempMat, new Scalar(LOWER_B, LOWER_G, LOWER_R), new Scalar(UPPER_B, UPPER_G, UPPER_R), color);
        // rgb - 012


        // Creates a submat based on the area of the box
        Mat area1 = color.submat(B1);
        Mat area2 = color.submat(B2);
        Mat area3 = color.submat(B3);
        // Finds the average color value of the channel in the rectangle
        double avg1 = Core.mean(area1).val[0];
        double avg2 = Core.mean(area2).val[0];
        double avg3 = Core.mean(area3).val[0];
        // Creates a visual representation to be seen on the camera
        Imgproc.rectangle(input, B1, new Scalar(0,0,255), 2);
        Imgproc.rectangle(input, B2, new Scalar(0,0,255), 2);
        Imgproc.rectangle(input, B3, new Scalar(0,0,255), 2);
        // Wherever the color is greatest, the object must be there
        // Finds the best color value, and makes a green border
        double max = Math.max(avg3, Math.max(avg1, avg2));
        if (max == avg1){
            pos = Pos.LEFT;
            Imgproc.rectangle(input, B1, new Scalar(0,255,0), 2);
        }
        else if (max == avg2){
            pos = Pos.CENTER;
            Imgproc.rectangle(input, B2, new Scalar(0,255,0), 2);
        }
        else if (max == avg3){
            pos = Pos.RIGHT;
            Imgproc.rectangle(input, B3, new Scalar(0,255,0), 2);
        }
        telemetry.addData("Position", pos);
        telemetry.addData("Box 1", avg1);
        telemetry.addData("Box 2", avg2);
        telemetry.addData("Box 3", avg3);
        telemetry.update();

        // Shows what the camera sees with rectangles
        return input;

    }
    // Returns the current position where the desired object is
    public Pos getPos(){
        return pos;
    }
}
