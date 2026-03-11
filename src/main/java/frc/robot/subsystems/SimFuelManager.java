package frc.robot.subsystems;

import org.ironmaple.simulation.IntakeSimulation;

public class SimFuelManager {
    private static SimFuelManager instance;

    private SimFuelManager(){}

    public synchronized static SimFuelManager getInstance(){
        if(instance == null){
            instance = new SimFuelManager();
        }
        return instance;
    }

    public IntakeSimulation intake;
}