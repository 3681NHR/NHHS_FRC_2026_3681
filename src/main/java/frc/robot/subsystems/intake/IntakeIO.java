package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public interface IntakeIO {

    default void updateInputs(IntakeIOInputs input) {}

    default void setRollerVoltage(Voltage voltage) {}

    default void setPivotGoal(Angle goal) {}
    default void setPivotVoltage(Voltage voltage) {}

    @AutoLog
    class IntakeIOInputs {
        // Roller
        public AngularVelocity rollerVelocity = RPM.zero();

        public Voltage rollerVoltageOut = Volts.zero();
        public Current rollerCurrentOut = Amps.zero();
        public Temperature rollerTemp = Celsius.zero();
        
        public boolean rollerConnected = false;

        // Pivot
        public Angle pivotAngle = Radians.zero();
        public AngularVelocity pivotVelocity = RadiansPerSecond.zero();

        public Voltage pivotVoltageOut = Volts.zero();
        public Current pivotCurrentOut = Amps.zero();
        public Temperature pivotTemp = Celsius.zero();

        public Angle pivotGoal = Radians.zero();
        public Angle pivotSetpointPos = Radians.zero();
        public AngularVelocity pivotSetpointVel = RadiansPerSecond.zero();
        public boolean pivotAtSetpoint = false;

        public boolean pivotOpenLoop = false;
        public boolean pivotMotorConnected = false;
        public boolean pivotEncoderConnected = false;
    }
}
