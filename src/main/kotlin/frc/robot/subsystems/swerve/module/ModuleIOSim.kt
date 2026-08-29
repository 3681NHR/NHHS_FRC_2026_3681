package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import frc.robot.constants.DriveConstants.Module
import frc.utils.BatteryVoltageSim
import frc.utils.SparkUtil
import frc.utils.controlWrappers.PID
import frc.utils.controlWrappers.ProfiledPID
import frc.utils.controlWrappers.SimpleFF
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.ironmaple.simulation.motorsims.SimulatedMotorController
import java.util.Arrays

open class ModuleIOSim(private val moduleSim: SwerveModuleSimulation) : ModuleIO {

    private val driveSim: SimulatedMotorController.GenericMotorController
    private val turnSim: SimulatedMotorController.GenericMotorController

    private var driveClosedLoop: Boolean = false
    private var turnClosedLoop: Boolean = false

    private val driveController = PID(Module.DRIVE_PID_SIM)
    private val turnController = ProfiledPID(Module.TURN_PID_SIM)

    private val driveFF = SimpleFF(Module.DRIVE_FF_SIM)

    private var driveAppliedVolts: Voltage = Volts.of(0.0)
    private var turnAppliedVolts: Voltage = Volts.of(0.0)

    private var turnGoal: Angle = Radians.of(0.0)
    private var driveGoal: AngularVelocity = RadiansPerSecond.of(0.0)

    private var turnPos: Angle = Radians.of(0.0)

    init {
        driveSim = moduleSim
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(40.0))
        turnSim = moduleSim
            .useGenericControllerForSteer()
            .withCurrentLimit(Amps.of(40.0))

        turnController.enableContinuousInput(Module.TURN_MIN_POS.`in`(Radians), Module.TURN_MAX_POS.`in`(Radians))

        BatteryVoltageSim.getInstance().addCurrentSource { moduleSim.driveMotorSupplyCurrent.`in`(Amps) }
        BatteryVoltageSim.getInstance().addCurrentSource { moduleSim.steerMotorSupplyCurrent.`in`(Amps) }
    }

    override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
        turnPos = Radians.of(MathUtil.inputModulus(moduleSim.steerAbsoluteFacing.radians, Module.TURN_MIN_POS.`in`(Radians), Module.TURN_MAX_POS.`in`(Radians)))
        turnGoal = Radians.of(MathUtil.inputModulus(turnGoal.`in`(Radians), Module.TURN_MIN_POS.`in`(Radians), Module.TURN_MAX_POS.`in`(Radians)))

        if (driveClosedLoop) {
            driveAppliedVolts = Volts.of(
                driveFF.calculate(driveGoal.`in`(RadiansPerSecond))
                    + driveController.calculate(moduleSim.driveWheelFinalSpeed.`in`(RadiansPerSecond), driveGoal.`in`(RadiansPerSecond))
            )
        } else {
            driveController.reset()
        }
        if (turnClosedLoop) {
            turnAppliedVolts = Volts.of(
                Module.TURN_FF_SIM.kS * Math.signum(turnController.setpoint.position)
                    + turnController.calculate(turnPos.`in`(Radians), turnGoal.`in`(Radians))
            )
        } else {
            turnController.reset(turnPos.`in`(Radians))
        }

        turnSim.requestVoltage(
            Volts.of(MathUtil.clamp(turnAppliedVolts.`in`(Volts), -RoboRioSim.getVInVoltage(), RoboRioSim.getVInVoltage()))
        )
        driveSim.requestVoltage(
            Volts.of(MathUtil.clamp(driveAppliedVolts.`in`(Volts), -RoboRioSim.getVInVoltage(), RoboRioSim.getVInVoltage()))
        )

        inputs.driveConnected = true
        inputs.drivePosition = moduleSim.driveWheelFinalPosition
        inputs.driveVelocity = moduleSim.driveWheelFinalSpeed
        inputs.driveAppliedVolts = driveAppliedVolts
        inputs.driveCurrent = Amps.of(Math.abs(moduleSim.driveMotorSupplyCurrent.`in`(Amps)))
        inputs.driveGoal = driveGoal
        inputs.driveSetpoint = RadiansPerSecond.of(driveController.setpoint)

        inputs.turnConnected = true
        inputs.turnPosition = turnPos
        inputs.turnVelocity = moduleSim.steerAbsoluteEncoderSpeed
        inputs.turnAppliedVolts = turnAppliedVolts
        inputs.turnCurrent = Amps.of(Math.abs(moduleSim.steerMotorSupplyCurrent.`in`(Amps)))
        inputs.turnGoal = turnGoal
        inputs.turnSetpoint = Radians.of(turnController.setpoint.position)

        inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps()
        inputs.odometryDrivePositionsRad = Arrays.stream(moduleSim.cachedDriveWheelFinalPositions).mapToDouble { it.`in`(Radians) }.toArray()
        inputs.odometryTurnPositionsRad = Arrays.stream(moduleSim.cachedSteerAbsolutePositions).mapToDouble { it.radians }.toArray()
    }

    override fun setDriveOpenLoop(output: Voltage) {
        driveClosedLoop = false
        driveAppliedVolts = output
    }

    override fun setTurnOpenLoop(output: Voltage) {
        turnClosedLoop = false
        turnAppliedVolts = output
    }

    override fun setDriveVelocity(velocity: AngularVelocity) {
        driveClosedLoop = true
        driveGoal = velocity
    }

    override fun setTurnPosition(rotation: Angle) {
        turnClosedLoop = true
        turnGoal = rotation
    }
}
