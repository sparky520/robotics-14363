package org.firstinspires.ftc.teamcode.SubSystems;

import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.states.outtakeStates;

public class PID {



    private double kp, ki, kd, ogP, ogI;
    private double totalError, lastError, lastTime;
    private ArrayList<Double> errors;
    private int target, max;
    private boolean isClose;
    private double I2CDeadzone = 30;

    private double iZone = 100;
    private double deadzone = 3;

    public PID(double kp, double ki, double kd, int target, int max) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        lastTime = 0;
        this.target = target;
        this.max = max;
        errors = new ArrayList<Double>();
    }

    public double getP(double error) {
        return kp * error;
    }

    public double getI(double error) {
        if (Math.abs(error) < 0.001) totalError = 0;
        return ki * totalError;
    }

    public double getD(double currentError) {
        return kd * (currentError - lastError);
    }

    public boolean checkCorrection(double error){

        if(Math.abs(errors.get(errors.size()-1)) < Math.abs(error))
            return true;
        else
            return false;

    }
    public void setP(double p){
        kp = p;
    }
    public void setI(double i){ ki = i;}

    public double getCorrection(double error) {
        totalError += error;

        double output = getP(error) + getI(error) + getD(error);

        lastError = error;
        errors.add(output);


        return output;
    }

    public void clearError(){
        totalError = 0;
    }

    public double getCorrectionPosition(double position, double voltage){
        setP(kp);

        double distance = Math.abs(position-target);

        if(distance < I2CDeadzone) isClose = true;
        else isClose = false;

        if(distance <= deadzone){
            setI(0);
            totalError = 0;
            return 0;
        }

        if((target == 0)){
            if(position <= 0) {
                setI(0);
                totalError = 0;
                return 0;
            }else{
                setI(ki*2);
            }
        }else if(distance <= iZone){
            setI(ki/2);
        }

        else setI(0);

        return getCorrection((target-position)/max);
    }

    public int getTarget(){
        return target;
    }

    public boolean isClose(){ return isClose;}

    public ArrayList<Double> aman(){
        return errors;
    }

    public void setTarget(int target){
        this.target = target;
    }

    public double getTotalError(){
        return totalError;
    }
}
