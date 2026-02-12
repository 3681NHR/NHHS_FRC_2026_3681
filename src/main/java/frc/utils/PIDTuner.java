package frc.utils;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.controlWrappers.ArmFF;
import frc.utils.controlWrappers.ElevatorFF;
import frc.utils.controlWrappers.PID;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.controlWrappers.SimpleFF;

public class PIDTuner {
    private PID pid;
    private ProfiledPID ppid;
    private SimpleFF ff;
    private ElevatorFF eff;
    private ArmFF aff;

    private LoggedNetworkNumber p;
    private LoggedNetworkNumber i;
    private LoggedNetworkNumber d;
    private LoggedNetworkNumber speed;
    private LoggedNetworkNumber accel;
    private LoggedNetworkNumber s;
    private LoggedNetworkNumber v;
    private LoggedNetworkNumber a;
    private LoggedNetworkNumber g;

    private LoggedNetworkBoolean apply;

    private String key = "";

    /**
     * contructs a PIDTuner
     * @param controller PID controller to tune
     * @param key NT4 key for tuner data and controls
     */
    public PIDTuner(PID controller, String key){
        this.pid = controller;
        this.key = key;

        p = new LoggedNetworkNumber(key + "/gains/Kp", 0);
        i = new LoggedNetworkNumber(key + "/gains/Ki", 0);
        d = new LoggedNetworkNumber(key + "/gains/Kd", 0);
        apply = new LoggedNetworkBoolean(key + "/gains/set", false);
    }
    /**
     * contructs a PIDTuner
     * @param controller PID controller to tune
     * @param key NT4 key for tuner data and controls
     */
    public PIDTuner(ProfiledPID controller, String key){
        this.ppid = controller;
        this.key = key;

        p = new LoggedNetworkNumber(key + "/gains/Kp", 0);
        i = new LoggedNetworkNumber(key + "/gains/Ki", 0);
        d = new LoggedNetworkNumber(key + "/gains/Kd", 0);
        speed = new LoggedNetworkNumber(key + "/gains/max speed", 0);
        accel = new LoggedNetworkNumber(key + "/gains/max accel", 0);
        apply = new LoggedNetworkBoolean(key + "/gains/set", false);
    }

    public PIDTuner withFF(ArmFF ff){
        this.aff = ff;
        
        s = new LoggedNetworkNumber(key + "/gains/Ks", 0);
        v = new LoggedNetworkNumber(key + "/gains/Kv", 0);
        a = new LoggedNetworkNumber(key + "/gains/Ka", 0);
        g = new LoggedNetworkNumber(key + "/gains/Kg", 0);

        return this;
    }
    public PIDTuner withFF(ElevatorFF ff){
        this.eff = ff;
        
        s = new LoggedNetworkNumber(key + "/gains/Ks", 0);
        v = new LoggedNetworkNumber(key + "/gains/Kv", 0);
        a = new LoggedNetworkNumber(key + "/gains/Ka", 0);
        g = new LoggedNetworkNumber(key + "/gains/Kg", 0);

        return this;
    }
    public PIDTuner withFF(SimpleFF ff){
        this.ff = ff;
        
        s = new LoggedNetworkNumber(key + "/gains/Ks", 0);
        v = new LoggedNetworkNumber(key + "/gains/Kv", 0);
        a = new LoggedNetworkNumber(key + "/gains/Ka", 0);

        return this;
    }

    public void update(double dt){
        if(DriverStation.isTest()){
            if(apply.get()){
                if(pid != null){
                    pid.setPID(
                        p.get(),
                        i.get(),
                        d.get()
                    );
                }
                if(ppid != null){
                    ppid.setPID(
                        p.get(),
                        i.get(),
                        d.get()
                    );
                    ppid.setConstraints(new Constraints(speed.get(), accel.get()));
                }
                if(ff != null){
                    ff.setKs(s.get());
                    ff.setKv(v.get());
                    ff.setKa(a.get());
                }
                if(aff != null){
                    aff.setKs(s.get());
                    aff.setKv(v.get());
                    aff.setKa(a.get());
                    aff.setKg(g.get());
                }
                if(eff != null){
                    eff.setKs(s.get());
                    eff.setKv(v.get());
                    eff.setKa(a.get());
                    eff.setKg(g.get());
                }
                apply.set(false);
            }
        }
    }
}