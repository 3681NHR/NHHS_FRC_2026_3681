package frc.utils;

import java.util.ArrayList;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.utils.controlWrappers.PID;

public class PIDTuner {
    private PID controller;

    private LoggedNetworkNumber p;
    private LoggedNetworkNumber i;
    private LoggedNetworkNumber d;

    private LoggedNetworkNumber s1;
    private LoggedNetworkNumber s2;
    private LoggedNetworkNumber timeout;
    private LoggedNetworkNumber maxStepKp;
    private LoggedNetworkNumber maxStepKi;
    private LoggedNetworkNumber maxStepKD;
    private LoggedNetworkNumber tolerance;

    private LoggedNetworkBoolean apply;
    private LoggedNetworkBoolean toggleTuner;

    private String key = "";

    private double overshoot = 0;
    private double KpMax;
    private double KiMax;
    private double KdMax;

    /**
     * contructs a PIDTuner
     * @param controller PID controller to tune
     * @param key NT4 key for tuner data and controls
     */
    public PIDTuner(PID controller, String key){
        this.controller = controller;
        this.key = key;

        p = new LoggedNetworkNumber(key + "/gains/Kp", 0);
        i = new LoggedNetworkNumber(key + "/gains/Ki", 0);
        d = new LoggedNetworkNumber(key + "/gains/Kd", 0);
        apply = new LoggedNetworkBoolean(key + "/gains/set", false);
        s1 = new LoggedNetworkNumber(key + "/auto/setpoint 1", 0);
        s2 = new LoggedNetworkNumber(key + "/auto/setpoint 2", 0);
        timeout = new LoggedNetworkNumber(key + "/auto/", 0);
        maxStepKp = new LoggedNetworkNumber(key + "/auto/max Step Kp(0 to disable tuning)", 0.1);
        maxStepKi = new LoggedNetworkNumber(key + "/auto/max Step Ki(0 to disable tuning)", 0);
        maxStepKD = new LoggedNetworkNumber(key + "/auto/max Step KD(0 to disable tuning)", 0.1);

        tolerance = new LoggedNetworkNumber(key + "/auto/setpoint tolerance");
    }


    public void update(){
        if(apply.get()){
            controller.setPID(
                p.get(),
                i.get(),
                d.get()
            );
        }

    }
}
