package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged in = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
        io.setPivotGoal(IntakeConstants.STOWED_ANGLE);
    }

    @Override
    public void periodic() {
        io.updateInputs(in);
        Logger.processInputs("IO/Intake", in);
        Logger.recordOutput("Subsystems/Intake/command",
                getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
    }

    //  Roller commands 

    /**
     * Closed-loop: spins the roller at the given velocity setpoint continuously.
     * The IO layer tracks the setpoint with ProfiledPID + SimpleFF each loop.
     */
    public Command velocityControl(AngularVelocity velocity) {
        return run(() -> io.setRollerVelocity(velocity))
                .withName("Roller Velocity Control");
    }

    /**
     * Open-loop fallback: applies a fixed voltage to the roller continuously.
     * Prefer {@link #velocityControl} for normal operation.
     */
    public Command voltageControl(Voltage voltage) {
        return run(() -> io.setRollerVoltage(voltage))
                .withName("Roller Voltage Control");
    }

    /**
     * Spins the roller at the configured intake velocity setpoint (closed-loop).
     */
    public Command intake() {
        return new ParallelCommandGroup(
                deploy(),
                velocityControl(IntakeConstants.INTAKE_VELOCITY)
                ).withName("Intake");
    }

    /**
     * Spins the roller in reverse at the configured eject velocity setpoint (closed-loop).
     */
    public Command eject() {
        return velocityControl(IntakeConstants.EJECT_VELOCITY)
                .withName("Eject");
    }

    /** Stops the roller immediately (one-shot, open-loop zero voltage). */
    public Command stopRoller() {
        return runOnce(() -> io.setRollerVoltage(Units.Volts.zero()))
                .withName("Stop Roller");
    }

    //  Pivot commands 

    /**
     * Deploys the intake pivot to the floor-facing position (closed-loop).
     * Returns immediately; the pivot tracks the goal in {@link #periodic}.
     */
    public Command deploy() {
        return runOnce(() -> io.setPivotGoal(IntakeConstants.DEPLOYED_ANGLE))
                .withName("Deploy Intake");
    }

    /**
     * Retracts the intake pivot to the stowed position (closed-loop).
     * Returns immediately; the pivot tracks the goal in {@link #periodic}.
     */
    public Command retract() {
        return new ParallelCommandGroup(runOnce(() -> io.setPivotGoal(IntakeConstants.STOWED_ANGLE)), velocityControl(Units.RPM.zero()))
                .withName("Retract Intake");
    }

    public Command defaultCommand() {
        return new ParallelCommandGroup(voltageControl(Units.Volts.zero()), runOnce(() -> io.setPivotVoltage(Units.Volts.zero())));
    }
}
