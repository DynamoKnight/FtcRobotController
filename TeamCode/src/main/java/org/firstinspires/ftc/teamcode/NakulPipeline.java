package org.firstinspires.ftc.teamcode;


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
public class NakulPipeline extends OpenCvPipeline {

    //////////////////////
    //CONSTRUCTOR
    //////////////////////
    public NakulPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    //////////////////////
    //VARIABLES
    //////////////////////
    Telemetry telemetry;
    // Used for Configuration in FTC Dashboard
    public static boolean returnInput = true;
    // Represents the different positions of the object
    enum Position {
        LEFT,
        CENTER,
        RIGHT
    }
    Position pos = Position.CENTER;
    // A Matrix stores image data
    Mat color = new Mat();
    // Rectangles
    public static Rect box1 = new Rect(30,120,30,30);
    public static Rect box2 = new Rect(145,120,30,30);
    public static Rect box3 = new Rect(260,120,30,30);

    // Sets the lower-bound and upper-bound RGB Range to search
    // Needs to be tuned to get best search

    // RED
    public static int lowerR = 110;
    public static int lowerG = 0;
    public static int lowerB = 0;

    public static int upperR = 255;
    public static int upperG = 90;
    public static int upperB = 255;

    // BLUE
    /*
    public static int lowerR = 0;
    public static int lowerG = 0;
    public static int lowerB = 65;

    public static int upperR = 70;
    public static int upperG = 255;
    public static int upperB = 255;
    */

    // WHITE
    /*
    public static int lowerR = 120;
    public static int lowerG = 130;
    public static int lowerB = 140;

    public static int upperR = 255;
    public static int upperG = 255;
    public static int upperB = 255;
    */

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
        Core.inRange(tempMat, new Scalar(lowerB, lowerG, lowerR), new Scalar(upperB, upperG, upperR), color);
        // rgb - 012

        /*
        // Creates a greyscale channel
        Mat grey = new Mat();
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
        Core.extractChannel(grey, color, 0);
        */

        // Creates a submat based on the area of the box
        Mat area1 = color.submat(box1);
        Mat area2 = color.submat(box2);
        Mat area3 = color.submat(box3);
        // Finds the average color value of the channel in the rectangle
        double avg1 = Core.mean(area1).val[0];
        double avg2 = Core.mean(area2).val[0];
        double avg3 = Core.mean(area3).val[0];
        // Creates a visual representation to be seen on the camera
        Imgproc.rectangle(input, box1, new Scalar(0,0,255), 2);
        Imgproc.rectangle(input, box2, new Scalar(0,0,255), 2);
        Imgproc.rectangle(input, box3, new Scalar(0,0,255), 2);
        // Wherever the color is greatest, the object must be there
        // Finds the best color value, and makes a green border
        double max = Math.max(avg3, Math.max(avg1, avg2));
        if (max == avg1){
            pos = Position.LEFT;
            Imgproc.rectangle(input, box1, new Scalar(0,255,0), 2);
        }
        else if (max == avg2){
            pos = Position.CENTER;
            Imgproc.rectangle(input, box2, new Scalar(0,255,0), 2);
        }
        else if (max == avg3){
            pos = Position.RIGHT;
            Imgproc.rectangle(input, box3, new Scalar(0,255,0), 2);
        }
        telemetry.addData("Position", pos);
        telemetry.addData("Box 1", avg1);
        telemetry.addData("Box 2", avg2);
        telemetry.addData("Box 3", avg3);
        telemetry.update();

        // Shows what the camera sees with rectangles
        if (returnInput){
            return input;
        }
        // Shows the greyscale channel
        else{
            return color;
        }
    }
    // Returns the current position where the desired object is
    public Position getPos(){
        return pos;
    }
}
