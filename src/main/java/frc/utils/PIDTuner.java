package frc.utils;

import java.util.ArrayList;
import java.util.function.DoubleConsumer;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.controlWrappers.ArmFF;
import frc.utils.controlWrappers.ElevatorFF;
import frc.utils.controlWrappers.PID;
import frc.utils.controlWrappers.PIDGains.Gains;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.controlWrappers.SimpleFF;

public class PIDTuner {

    private LoggedNetworkNumber p;
    private LoggedNetworkNumber i;
    private LoggedNetworkNumber d;
    private LoggedNetworkNumber speed;
    private LoggedNetworkNumber accel;
    private LoggedNetworkNumber s;
    private LoggedNetworkNumber v;
    private LoggedNetworkNumber a;
    private LoggedNetworkNumber g;

    private DoubleConsumer setp;
    private DoubleConsumer seti;
    private DoubleConsumer setd;
    private DoubleConsumer setspeed;
    private DoubleConsumer setaccel;
    private DoubleConsumer sets;
    private DoubleConsumer setv;
    private DoubleConsumer seta;
    private DoubleConsumer setg;

    private LoggedNetworkBoolean apply;

    private String key = "";

    /**
     * contructs a PIDTuner
     * @param controller PID controller to tune
     * @param key NT4 key for tuner data and controls
     */
    public PIDTuner(PID controller, String key){
        this.key = key;

        p = new LoggedNetworkNumber(key + "/gains/Kp", 0);
        i = new LoggedNetworkNumber(key + "/gains/Ki", 0);
        d = new LoggedNetworkNumber(key + "/gains/Kd", 0);
        apply = new LoggedNetworkBoolean(key + "/gains/set", false);

        setp = controller::setP;
        seti = controller::setI;
        setd = controller::setD;
    }
    /**
     * contructs a PIDTuner
     * @param controller PID controller to tune
     * @param key NT4 key for tuner data and controls
     */
    public PIDTuner(ProfiledPID controller, String key){
        this.key = key;

        p = new LoggedNetworkNumber(key + "/gains/Kp", 0);
        i = new LoggedNetworkNumber(key + "/gains/Ki", 0);
        d = new LoggedNetworkNumber(key + "/gains/Kd", 0);
        speed = new LoggedNetworkNumber(key + "/gains/max speed", 0);
        accel = new LoggedNetworkNumber(key + "/gains/max accel", 0);

        apply = new LoggedNetworkBoolean(key + "/gains/set", false);
        
        setp = controller::setP;
        seti = controller::setI;
        setd = controller::setD;
        setspeed = (speed) -> controller.setSpeed(speed);
        setaccel = (accel) -> controller.setAccel(accel);
    }
    public PIDTuner(DoubleConsumer p,DoubleConsumer i,DoubleConsumer d,DoubleConsumer speed, DoubleConsumer accel, String key){
        this.key = key;

        this.p = new LoggedNetworkNumber(key + "/gains/Kp", 0);
        this.i = new LoggedNetworkNumber(key + "/gains/Ki", 0);
        this.d = new LoggedNetworkNumber(key + "/gains/Kd", 0);
        this.speed = new LoggedNetworkNumber(key + "/gains/max speed", 0);
        this.accel = new LoggedNetworkNumber(key + "/gains/max accel", 0);

        apply = new LoggedNetworkBoolean(key + "/gains/set", false);
        
        setp = p;
        seti = i;
        setd = d;
        setspeed = speed;
        setaccel = accel;
    }
    public PIDTuner(DoubleConsumer p,DoubleConsumer i,DoubleConsumer d, String key){
        this.key = key;

        this.p = new LoggedNetworkNumber(key + "/gains/Kp", 0);
        this.i = new LoggedNetworkNumber(key + "/gains/Ki", 0);
        this.d = new LoggedNetworkNumber(key + "/gains/Kd", 0);

        apply = new LoggedNetworkBoolean(key + "/gains/set", false);
        
        setp = p;
        seti = i;
        setd = d;
    }

    public PIDTuner withFF(ArmFF ff){
        
        s = new LoggedNetworkNumber(key + "/gains/Ks", 0);
        v = new LoggedNetworkNumber(key + "/gains/Kv", 0);
        a = new LoggedNetworkNumber(key + "/gains/Ka", 0);
        g = new LoggedNetworkNumber(key + "/gains/Kg", 0);
        apply = new LoggedNetworkBoolean(key + "/gains/set", false);

        sets = ff::setKs;
        setv = ff::setKv;
        seta = ff::setKa;
        setg = ff::setKg;

        return this;
    }
    public PIDTuner withFF(ElevatorFF ff){
        
        s = new LoggedNetworkNumber(key + "/gains/Ks", 0);
        v = new LoggedNetworkNumber(key + "/gains/Kv", 0);
        a = new LoggedNetworkNumber(key + "/gains/Ka", 0);
        g = new LoggedNetworkNumber(key + "/gains/Kg", 0);
        apply = new LoggedNetworkBoolean(key + "/gains/set", false);

        sets = ff::setKs;
        setv = ff::setKv;
        seta = ff::setKa;
        setg = ff::setKg;

        return this;
    }
    public PIDTuner withFF(SimpleFF ff){
        
        s = new LoggedNetworkNumber(key + "/gains/Ks", 0);
        v = new LoggedNetworkNumber(key + "/gains/Kv", 0);
        a = new LoggedNetworkNumber(key + "/gains/Ka", 0);
        apply = new LoggedNetworkBoolean(key + "/gains/set", false);
        
        sets = ff::setKs;
        setv = ff::setKv;
        seta = ff::setKa;

        return this;
    }
    public PIDTuner withFF(DoubleConsumer s,DoubleConsumer v,DoubleConsumer a){
        
        this.s = new LoggedNetworkNumber(key + "/gains/Ks", 0);
        this.v = new LoggedNetworkNumber(key + "/gains/Kv", 0);
        this.a = new LoggedNetworkNumber(key + "/gains/Ka", 0);
        apply = new LoggedNetworkBoolean(key + "/gains/set", false);

        sets = s;
        setv = v;
        seta = a;

        return this;
    }
    public PIDTuner withFF(DoubleConsumer s,DoubleConsumer v,DoubleConsumer a,DoubleConsumer g){
        
        this.s = new LoggedNetworkNumber(key + "/gains/Ks", 0);
        this.v = new LoggedNetworkNumber(key + "/gains/Kv", 0);
        this.a = new LoggedNetworkNumber(key + "/gains/Ka", 0);
        this.g = new LoggedNetworkNumber(key + "/gains/Kg", 0);
        apply = new LoggedNetworkBoolean(key + "/gains/set", false);

        sets = s;
        setv = v;
        seta = a;
        setg = g;

        return this;
    }

    public void update(double dt){
        if(DriverStation.isTest()){
            if(apply.get()){
                if(setp != null) setp.accept(p.get());
                if(seti != null) seti.accept(i.get());
                if(setd != null) setd.accept(d.get());

                if(setspeed != null) setspeed.accept(speed.get());
                if(setaccel != null) setaccel.accept(accel.get());

                if(sets != null) sets.accept(s.get());
                if(setv != null) setv.accept(v.get());
                if(seta != null) seta.accept(a.get());
                if(setg != null) setg.accept(g.get());
                apply.set(false);
            }
        }
    }

    public static ArrayList<Gains> tunableGains = new ArrayList<Gains>();
    public static void updateTunables(){
        for(Gains g : tunableGains){
            g.update();
        }
    }
}