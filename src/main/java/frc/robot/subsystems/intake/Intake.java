package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.IntakeConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged in = new IntakeIOInputsAutoLogged();
    private boolean isIntaking = false;
    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(in);
        Logger.processInputs("IO/Intake", in);
        Logger.recordOutput("Subsystems/Intake/command", getCurrentCommand() == null ? "none" : getCurrentCommand().getName());

        if(DriverStation.isDisabled()){
            io.setPivotGoal(in.pivotAngle);
        }
    }

    public Command agitate(){
        return Commands.repeatingSequence(
                Commands.waitUntil(() -> !isIntaking),
                Commands.runOnce(() -> io.setPivotGoal(INTAKE_PIVOT_AGITATE_ANGLE), this),
                Commands.waitUntil(() -> in.pivotAtSetpoint),
                Commands.runOnce(() -> io.setPivotGoal(INTAKE_DEPLOYED_ANGLE), this),
                Commands.waitUntil(() -> in.pivotAtSetpoint)
        )
        .withName("agitate");
    }


    public Command intake(){
        return Commands.run(() -> {
            isIntaking = true;
            io.setPivotGoal(INTAKE_DEPLOYED_ANGLE);
            io.setRollerVoltage(INTAKE_RUN_VOLTAGE);
        }, this)
        .finallyDo(() -> {
            io.setRollerVoltage(Volts.of(0));
            isIntaking = false;
//            io.setPivotGoal(INTAKE_STOWED_ANGLE);
        })
        .withName("intake");
    }
    public Command uptake() {
        return Commands.runOnce(() -> {
            io.setRollerVoltage(Volts.zero());
            io.setPivotGoal(INTAKE_STOWED_ANGLE);
        }, this).withName("up up up in the bot");
    }


    
    public Command manualControl(Supplier<Angle> pivot, Supplier<Voltage> roller){
        
        return Commands.run(() -> {
            io.setPivotGoal(pivot.get());
            io.setRollerVoltage(roller.get());
        }, this)
        .finallyDo(() -> io.setRollerVoltage(Volts.of(0)))
        .withName("manual");
    }
    
    public Command voltageControl(Supplier<Voltage> pivot, Supplier<Voltage> roller){
        
        return Commands.run(() -> {
            io.setPivotVoltage(pivot.get());
            io.setRollerVoltage(roller.get());
        }, this)
        .finallyDo(() -> io.setRollerVoltage(Volts.of(0)))
        .withName("manual voltage");
    }

    public Angle getAngle(){
        return in.pivotAngle;
    }
}
