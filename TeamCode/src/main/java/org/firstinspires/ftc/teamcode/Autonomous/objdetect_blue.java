package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class objdetect_blue extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        MIDDLE,
        NOT_FOUND
    }
    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(140, 35),
            new Point(200, 75));
    static double PERCENT_COLOR_THRESHOLD = 0.4;
    public objdetect_blue(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

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

        boolean TSELeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean TSEMiddle = middleValue > PERCENT_COLOR_THRESHOLD;

        if(TSELeft && TSEMiddle){
            location = Location.NOT_FOUND;
            telemetry.addData("TSE Location", "not found");
            // TSE = team scoring element
        }
        if (TSELeft){
            location = Location.MIDDLE;
            telemetry.addData("TSE Location", "MIDDLE");
        }
        else{
            location = Location.LEFT;
            telemetry.addData("TSE Location", "left");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar noTSE = new Scalar(255, 0, 0);
        Scalar yesTSE = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? yesTSE:noTSE);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? yesTSE:noTSE);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
