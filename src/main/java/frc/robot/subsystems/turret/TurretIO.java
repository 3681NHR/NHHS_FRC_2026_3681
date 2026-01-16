package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    public default void updateInputs(TurretIOInputs input){}

    public default void setGoal(double goal){}
    public default void setVout(double vout){}
    public default void setOpenLoop(boolean openloop){}
    
    @AutoLog
    public class TurretIOInputs{

        public double calculatedAngle = 0;
        public double calculatedSpeed = 0;

        public double motorVoltageOut = 0;
        public double motorCurrentOut = 0;
        public double motorTemp = 0;

        public double rawAngleE1 = 0;
        public double rawAngleE2 = 0;
    }
}
