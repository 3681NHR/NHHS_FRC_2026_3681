package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.DriveConstants.*;

import org.littletonrobotics.junction.Logger;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
        driveDisconnectedAlert = new Alert(
                "Disconnected drive motor on module " + Integer.toString(index) + ".",
                AlertType.kError);
        turnDisconnectedAlert = new Alert(
                "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IO/Drive/Module" + Integer.toString(index), inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTurnTimestamps.length + inputs.odometryDriveTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            Distance positionMeters = module.WHEEL_RAD;//.times(inputs.odometryDrivePositionsRad[i]);
            Rotation2d angle = new Rotation2d(inputs.odometryTurnPositionsRad[i]);
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
    }

    /**
     * Runs the module with the specified setpoint state. Mutates the state to
     * optimize it.
     */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(getAngle());
        // state.cosineScale(new Rotation2d(inputs.turnPositionRad));
        state = newCosineScale(state, new Rotation2d(inputs.turnPosition));

        // Apply setpoints
        io.setDriveVelocity(RadiansPerSecond.of(state.speedMetersPerSecond / module.WHEEL_RAD.in(Meters)));
        io.setTurnPosition(Radians.of(state.angle.getRadians()));
    }
    
    public static SwerveModuleState newCosineScale(SwerveModuleState state, Rotation2d currentAngle) {
        double angleDiff = state.angle.minus(currentAngle).getRadians();
        //use cos()^3 to reduce motion untill closer aligned
        double scale = Math.pow(Math.abs(Math.cos(angleDiff))+0.004, 3);//|cos(angleDiff)|^3
        scale = MathUtil.clamp(scale, -1, 1);//mult is negitive when angleDiff is > 90 degrees

        double scaledSpeed = state.speedMetersPerSecond * scale;
        return new SwerveModuleState(scaledSpeed, state.angle);
    }

    /**
     * Runs the module with the specified output while controlling to zero degrees.
     */
    public void runCharacterization(Voltage output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(Radians.of(0));
    }

    /** spin with no drive speed */
    public void runSteerCharacterization(Voltage output) {
        io.setDriveOpenLoop(Volts.of(0));
        io.setTurnOpenLoop(output);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setDriveOpenLoop(Volts.of(0.0));
        io.setTurnOpenLoop(Volts.of(0.0));
    }
    /**
     * Holds the module at the last setpoint.
     */
    public void idle(){
        io.setDriveVelocity(inputs.driveVelocity);
        io.setTurnPosition(inputs.turnPosition);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return new Rotation2d(inputs.turnPosition);
    }

    /** Returns the current drive position of the module in meters. */
    public Distance getDrivePosition() {
        return Meters.of(inputs.drivePosition.in(Radians) * module.WHEEL_RAD.in(Meters));
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond.of(inputs.driveVelocity.in(RadiansPerSecond) * module.WHEEL_RAD.in(Meters));
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }
}