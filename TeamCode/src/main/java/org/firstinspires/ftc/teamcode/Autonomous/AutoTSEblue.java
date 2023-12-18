package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "TSE blue Detector", group = "Auto")
public class AutoTSEblue extends LinearOpMode {
    OpenCvCamera roboCam;
    String webcamName;
    @Override
    public void runOpMode() throws InterruptedException{
        webcamName = "Webcam 1";
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        roboCam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        objdetect_blue detector = new objdetect_blue(telemetry);
        roboCam.setPipeline(detector);
        roboCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                roboCam.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("error opening cam");
                telemetry.update();
            }
        });

        waitForStart();

    }
}
