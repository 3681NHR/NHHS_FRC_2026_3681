package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.DriveConstants.module;
import frc.utils.BatteryVoltageSim;
import frc.utils.SparkUtil;
import frc.utils.controlWrappers.PID;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.controlWrappers.SimpleFF;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {

    private final SwerveModuleSimulation moduleSim;

    private final SimulatedMotorController.GenericMotorController driveSim;
    private final SimulatedMotorController.GenericMotorController turnSim;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;

    private PID driveController = new PID(module.DRIVE_PID_SIM);
    private ProfiledPID turnController = new ProfiledPID(module.TURN_PID_SIM);

    private final SimpleFF driveFF = new SimpleFF(module.DRIVE_FF_SIM);

    private Voltage driveAppliedVolts = Volts.of(0.0);
    private Voltage turnAppliedVolts = Volts.of(0.0);

    private Angle turnGoal = Radians.of(0.0);
    private AngularVelocity driveGoal = RadiansPerSecond.of(0.0);

    private Angle turnPos = Radians.of(0.0);

    public ModuleIOSim(SwerveModuleSimulation moduleSim) {
        this.moduleSim = moduleSim;
        // Create drive and turn sim models
        driveSim = moduleSim
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(Amps.of(40));
        turnSim = moduleSim
                .useGenericControllerForSteer()
                .withCurrentLimit(Amps.of(40));

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(module.TURN_MIN_POS.in(Radians), module.TURN_MAX_POS.in(Radians));

        BatteryVoltageSim.getInstance().addCurrentSource(() -> moduleSim.getDriveMotorSupplyCurrent().in(Amps));
        BatteryVoltageSim.getInstance().addCurrentSource(() -> moduleSim.getSteerMotorSupplyCurrent().in(Amps));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        turnPos = Radians.of(MathUtil.inputModulus(moduleSim.getSteerAbsoluteFacing().getRadians(), module.TURN_MIN_POS.in(Radians), module.TURN_MAX_POS.in(Radians)));
        turnGoal = Radians.of(MathUtil.inputModulus(turnGoal.in(Radians), module.TURN_MIN_POS.in(Radians), module.TURN_MAX_POS.in(Radians)));

        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts = Volts.of(driveFF.calculate(driveGoal.in(RadiansPerSecond))
                    + driveController.calculate(moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond), driveGoal.in(RadiansPerSecond)));
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = Volts.of(module.TURN_FF_SIM.kS() * Math.signum(turnController.getSetpoint().position)
                    + turnController.calculate(turnPos.in(Radians), turnGoal.in(Radians)));
        } else {
            turnController.reset(turnPos.in(Radians));
        }

        // Update simulation state
        turnSim.requestVoltage(
                Volts.of(MathUtil.clamp(turnAppliedVolts.in(Volts), -RoboRioSim.getVInVoltage(), RoboRioSim.getVInVoltage())));
        driveSim.requestVoltage(
                Volts.of(MathUtil.clamp(driveAppliedVolts.in(Volts), -RoboRioSim.getVInVoltage(), RoboRioSim.getVInVoltage())));

        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePosition = moduleSim.getDriveWheelFinalPosition();
        inputs.driveVelocity = moduleSim.getDriveWheelFinalSpeed();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrent = Amps.of(Math.abs(moduleSim.getDriveMotorSupplyCurrent().in(Amps)));

        // Update turn inputs
        inputs.turnConnected = true;
        inputs.turnPosition = turnPos;
        inputs.turnVelocity = moduleSim.getSteerAbsoluteEncoderSpeed();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrent = Amps.of(Math.abs(moduleSim.getSteerMotorSupplyCurrent().in(Amps)));

        // Update odometry inputs
        inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
        inputs.odometryDrivePositions = Arrays.stream(moduleSim.getCachedDriveWheelFinalPositions()).mapToDouble(r -> r.in(Radians)).toArray();
        inputs.odometryTurnPositions = Arrays.stream(moduleSim.getCachedSteerAbsolutePositions()).mapToDouble(r -> r.getRadians()).toArray();
    }

    @Override
    public void setDriveOpenLoop(Voltage output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(Voltage output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(AngularVelocity velocity) {
        driveClosedLoop = true;
        driveGoal = velocity;
    }

    @Override
    public void setTurnPosition(Angle rotation) {
        turnClosedLoop = true;
        turnGoal = rotation;
    }
}