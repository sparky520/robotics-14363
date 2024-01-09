package org.firstinspires.ftc.teamcode.drive.autoExecutable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.teamcode.states.*;
import org.openftc.easyopencv.OpenCvPipeline;

public class blueObjectDetect extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    private String location = "RIGHT";

    static final Rect LEFT_ROI = new Rect(
            new Point(60, 110),
            new Point(120, 150));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(220, 110),
            new Point(280, 150));
    static double MIDDLE_PERCENT_COLOR_THRESHOLD = 0.1;
    static double LEFT_PERCENT_COLOR_THRESHOLD = 0.2;
    public blueObjectDetect(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(100, 150, 0);
        Scalar highHSV = new Scalar(140, 255,255);


        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;

        left.release();
        middle.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");

        boolean TSELeft = leftValue > LEFT_PERCENT_COLOR_THRESHOLD;
        boolean TSEMiddle = middleValue > MIDDLE_PERCENT_COLOR_THRESHOLD;

        if(TSEMiddle){
            location = "MIDDLE";
            telemetry.addData("TSE Location", "MIDDLE");
            // TSE = team scoring element
        }
        else if (TSELeft){
            location = "LEFT";
            telemetry.addData("TSE Location", "LEFT");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar noTSE = new Scalar(255, 0, 0);
        Scalar yesTSE = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location.equals("LEFT")? yesTSE:noTSE);
        Imgproc.rectangle(mat, MIDDLE_ROI, location.equals("MIDDLE")? yesTSE:noTSE);

        return mat;
    }

    public String getLocation() {
        return location;
    }
}
