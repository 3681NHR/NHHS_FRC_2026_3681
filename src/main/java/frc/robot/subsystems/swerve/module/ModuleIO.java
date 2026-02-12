package frc.robot.subsystems.swerve.module;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public boolean driveOpenLoop = false;
        public Angle drivePosition = Radians.of(0);
        public AngularVelocity driveVelocity = RadiansPerSecond.of(0);
        public AngularVelocity driveGoal = RadiansPerSecond.of(0);
        public AngularVelocity driveSetpoint = RadiansPerSecond.of(0);
        public Voltage driveAppliedVolts = Volts.of(0);
        public Current driveCurrent = Amps.of(0);
        public Temperature driveTemp = Celsius.of(0);

        public boolean turnConnected = false;
        public boolean turnOpenLoop = false;
        public Angle turnPosition = Radians.of(0);
        public Angle turnGoal = Radians.of(0);
        public Angle turnSetpoint = Radians.of(0);
        public AngularVelocity turnVelocity = RadiansPerSecond.of(0);
        public Voltage turnAppliedVolts = Volts.of(0);
        public Current turnCurrent = Amps.of(0);
        public Temperature turnTemp = Celsius.of(0);

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public double[] odometryTurnPositionsRad = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {
    }

    /** Run the drive motor at the specified open loop value. */
    public default void setDriveOpenLoop(Voltage output) {
    }

    /** Run the turn motor at the specified open loop value. */
    public default void setTurnOpenLoop(Voltage output) {
    }

    /** Run the drive motor at the specified velocity. */
    public default void setDriveVelocity(AngularVelocity velocity) {
    }

    /** Run the turn motor to the specified rotation. */
    public default void setTurnPosition(Angle rotation) {
    }
}