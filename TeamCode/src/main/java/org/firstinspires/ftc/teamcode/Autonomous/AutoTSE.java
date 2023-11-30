package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "TSE Detector", group = "Auto")
public class AutoTSE extends LinearOpMode {
    OpenCvCamera RoboCam;
    @Override
    public void runOpMode() throws InterruptedException{
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        RoboCam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        objdetect_blue detector = new objdetect_blue(telemetry);
        RoboCam.setPipeline(detector);
        RoboCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                RoboCam.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("error opening cam");
                telemetry.update();
            }
        });
    }
}
