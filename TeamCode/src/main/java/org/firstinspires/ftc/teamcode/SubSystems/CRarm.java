package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.states.armState;

public class CRarm {
    CRServo leftArm,rightArm;
    ElapsedTime timer = new ElapsedTime();
    Telemetry telemetryServo;
    ServoController leftController, rightController;

    public CRarm(HardwareMap hardwareMap, Telemetry telemetry){
        leftArm = hardwareMap.get(CRServo.class, "leftArm");
        rightArm = hardwareMap.get(CRServo.class, "rightArm");
        leftController = leftArm.getController();
        rightController = rightArm.getController();
        telemetryServo = telemetry;
    }

    public void setPos(double power, double time){
        leftArm.setPower(power);
        rightArm.setPower(power);
        timer.reset();
        while (timer.seconds() < time){
            telemetryServo.addLine(leftController.getServoPosition(leftArm.getPortNumber())+ "");
            telemetryServo.addLine(rightController.getServoPosition(rightArm.getPortNumber())+ "");
            telemetryServo.update();
        }
        leftArm.setPower(0);
        rightArm.setPower(0);
    }

    public void testSet(){
        leftController.setServoPosition(leftArm.getPortNumber(),.5);
        rightController.setServoPosition(rightArm.getPortNumber(),.5);
    }
}
