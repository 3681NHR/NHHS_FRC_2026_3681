package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface ClimberIO {
    public default void updateInputs(ClimberIOInputs input){}

    public default void setVoltage(Voltage voltage){}
    public default void setGoal(Distance goal){}
    
    public default void setPosition(Distance goal){}
    public default void setHomed(boolean homed){}
    @AutoLog
    public class ClimberIOInputs{
        public Distance position = Meters.zero();
        public LinearVelocity velocity = MetersPerSecond.zero();

        public Voltage motorVoltageOut = Volts.zero();
        public Current motorCurrentOut = Amps.zero();
        public Temperature motorTemp = Celsius.zero();

        public Distance goal = Meters.zero();

        public LinearVelocity velocitySetpoint = MetersPerSecond.zero();
        public Distance positionSetpoint = Meters.zero();
        public boolean atSetpoint = false;

        public boolean connected = false;
        public boolean openLoop = false;
        boolean homed = false;
    }

    
}
