package frc.utils.rumble;

import java.util.ArrayList;
import java.util.Collection;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class RumbleHandler {
    private XboxController controller;
    private double port;

    private boolean following = false;
    private RumbleHandler lead;

    private ArrayList<Rumble> queue = new ArrayList<Rumble>();
    public RumbleHandler(XboxController controller){
        this.controller = controller;
        this.port = controller.getPort();
    }
    public RumbleHandler(RumbleHandler lead){
        this.following = true;
        this.lead = lead;
    }
    /**
     * clear rumble queue, effectivly stopping all rumble
     */
    public void clearQue(){
        queue.clear();
    }
    public void addToQue(Rumble[] a){
        for(Rumble b : a){
            queue.add(b);
        }
    }
    public void addToQue(Collection<Rumble> a){
        for(Rumble b : a){
            queue.add(b);
        }
    }
    public void addToQue(Rumble a){
        queue.add(a);
    }

    public void overrideQue(Rumble a){
        clearQue();
        queue.add(a);
    }
    public void overrideQue(Collection<Rumble> a){
        clearQue();
        for(Rumble b : a){
            queue.add(b);
        }
    }
    public void overrideQue(Rumble[] a){
        clearQue();
        for(Rumble b : a){
            queue.add(b);
        }
    }
    public void update(double loopTime){
        if(following){
            queue = lead.queue;
        }
        if(queue.size() > 0){
            queue.get(0).time -= loopTime;
        }
        for (int i=0; i < queue.size(); i++) {
            if(queue.get(i).time <= 0){
                queue.remove(i);
            }
        }
        if(queue.size() >= 1){
            controller.setRumble(RumbleType.kLeftRumble, queue.get(0).powL);
            controller.setRumble(RumbleType.kRightRumble, queue.get(0).powR);
            Logger.recordOutput("haptics/rumble: "+port+"/current Strength left", queue.get(0).powL);
            Logger.recordOutput("haptics/rumble: "+port+"/current Strength right", queue.get(0).powR);
            Logger.recordOutput("haptics/rumble: "+port+"/queue", getPows());
        } else {
            controller.setRumble(RumbleType.kBothRumble, 0);
            Logger.recordOutput("haptics/rumble: "+port+"/current Strength left", 0.0);
            Logger.recordOutput("haptics/rumble: "+port+"/current Strength right", 0.0);
            Logger.recordOutput("haptics/rumble: "+port+"/queue", new double[0][0]);
        }
        Logger.recordOutput("haptics/rumble: "+port+"/following", following);
        
        if(following){
            Logger.recordOutput("haptics/rumble: "+port+"/following rumble", lead.port);
        }
    } 
    private double[][] getPows(){
        double[][] pows = new double[queue.size()][3];
        for(int i=0; i < queue.size(); i++){
            Rumble d = queue.get(i);
            pows[i][0] = d.powR;
            pows[i][1] = d.powL;
            pows[i][2] = d.time;
        }
        return pows;
    }
}