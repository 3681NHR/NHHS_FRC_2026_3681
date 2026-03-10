package frc.robot.subsystems;

public class SimFuelManager {
    private static SimFuelManager instance;

    private SimFuelManager(){}

    public synchronized static SimFuelManager getInstance(){
        if(instance != null){
            instance = new SimFuelManager();
        }
        return instance;
    }
}