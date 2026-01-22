package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public interface TurretIO {

    public default void updateInputs(TurretIOInputs input){}

    public default void setGoal(double goal){}
    public default void setVout(double vout){}
    public default void setOpenLoop(boolean openloop){}
    
    @AutoLog
    public class TurretIOInputs{

        public double filteredAngle = 0.0;
        public double filteredSpeed = 0.0;
        public double rawAngle = 0.0;
        public double rawSpeed = 0.0;

        public double motorVoltageOut = 0;
        public double motorCurrentOut = 0;
        public double motorTemp = 0;

        public double goal = 0.0;
        public State setpoint = new State();
        public boolean atSetpoint = false;

        public boolean openLoop = false;

        public Rotation2d angleE1 = new Rotation2d();
        public Rotation2d angleE2 = new Rotation2d();
    }
}
