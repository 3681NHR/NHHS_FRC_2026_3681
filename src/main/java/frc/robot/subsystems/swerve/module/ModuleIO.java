package frc.robot.subsystems.swerve.module;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected;
        public Angle drivePosition;
        public AngularVelocity driveVelocity;
        public Voltage driveAppliedVolts;
        public Current driveCurrent;
        public Temperature driveTemp;

        public boolean turnConnected;
        public Angle turnPosition;
        public AngularVelocity turnVelocity;
        public Voltage turnAppliedVolts;
        public Current turnCurrent;
        public Temperature turnTemp;

        public Time[] odometryTimestamps = new Time[] {};
        public Angle[] odometryDrivePositions = new Angle[] {};
        public Angle[] odometryTurnPositions = new Angle[] {};
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