package frc.robot.subsystems.intake

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.LinearSystemSim
import frc.robot.constants.Constants
import frc.robot.constants.IntakeConstants.INTAKE_DEPLOYED_ANGLE
import frc.robot.constants.IntakeConstants.INTAKE_PIVOT_FF_GAINS
import frc.robot.constants.IntakeConstants.INTAKE_PIVOT_ID_GAINS
import frc.robot.constants.IntakeConstants.INTAKE_PIVOT_MAX_ANGLE
import frc.robot.constants.IntakeConstants.INTAKE_PIVOT_MIN_ANGLE
import frc.robot.constants.IntakeConstants.INTAKE_PIVOT_PID_GAINS
import frc.robot.constants.IntakeConstants.INTAKE_PIVOT_TOLERANCE
import frc.robot.constants.IntakeConstants.INTAKE_STOWED_ANGLE
import frc.robot.subsystems.SimFuelManager
import frc.utils.ExtraMath
import frc.utils.controlWrappers.ArmFF
import frc.utils.controlWrappers.ProfiledPID
import org.ironmaple.simulation.IntakeSimulation
import org.ironmaple.simulation.IntakeSimulation.IntakeSide
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation
import org.littletonrobotics.junction.Logger

class IntakeIOSim(
    drive: AbstractDriveTrainSimulation
) : IntakeIO {

    // Roller
    private var rollerVout: Voltage = Volts.zero()

    // Pivot
    private val pivotPID = ProfiledPID(INTAKE_PIVOT_PID_GAINS)
    private var pivotFF = ArmFF(INTAKE_PIVOT_FF_GAINS)

    private var pivotOpenLoop = false
    private var pivotGoal: Angle = INTAKE_STOWED_ANGLE

    private var pivotVout: Voltage = Volts.zero()

    private val model: LinearSystem<N2, N1, N2> = LinearSystemId.identifyPositionSystem(INTAKE_PIVOT_ID_GAINS.kV, INTAKE_PIVOT_ID_GAINS.kA)
    private val sim = LinearSystemSim<N2, N1, N2>(model, 0.0, 0.0)

    private val mapleSimIntake: IntakeSimulation

    init {
        // Live-tuning callbacks
        INTAKE_PIVOT_PID_GAINS.withCallback { pivotPID.setGains(INTAKE_PIVOT_PID_GAINS) }
        INTAKE_PIVOT_FF_GAINS.withCallback {
            pivotFF.setKs(INTAKE_PIVOT_FF_GAINS.kS)
            pivotFF.setKg(INTAKE_PIVOT_FF_GAINS.kG)
            pivotFF.setKv(INTAKE_PIVOT_FF_GAINS.kV)
            pivotFF.setKa(INTAKE_PIVOT_FF_GAINS.kA)
        }

        pivotPID.setTolerance(INTAKE_PIVOT_TOLERANCE.`in`(Units.Radians))

        mapleSimIntake = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            drive,
            Inches.of(25.0),
            Inches.of(6.0),
            IntakeSide.FRONT,
            20
        )

        SimFuelManager.getInstance().intake = mapleSimIntake
    }

    override fun updateInputs(input: IntakeIO.IntakeIOInputs) {
        sim.update(Constants.EVENT_LOOP_TIME)

        // TODO, kv from recalc, test on bot
        input.rollerVelocity = RPM.of(124.075 * rollerVout.`in`(Volts))

        input.rollerVoltageOut = rollerVout

        input.rollerConnected = true

        // Pivot closed-loop
        if (!pivotOpenLoop) {
            pivotVout = Volts.of(pivotPID.calculate(sim.output.get(0, 0), pivotGoal.`in`(Units.Radians)))
            pivotVout = pivotVout.plus(Volts.of(pivotFF.calculate(sim.output.get(0, 0), pivotPID.setpoint.velocity)))
        }
        pivotVout = Volts.of(
            pivotVout.`in`(Volts)
                - ExtraMath.lesser(INTAKE_PIVOT_ID_GAINS.kS * Math.signum(sim.output.get(1, 0)), pivotVout.`in`(Volts))
                - INTAKE_PIVOT_ID_GAINS.kG * Math.cos(sim.output.get(0, 0))
        )
        if (Radians.of(sim.output.get(0, 0)).gt(INTAKE_PIVOT_MAX_ANGLE)) {
            sim.setState(VecBuilder.fill(INTAKE_PIVOT_MAX_ANGLE.`in`(Radians), 0.0))
            sim.setInput(MathUtil.clamp(pivotVout.`in`(Volts), -12.0, 0.0))
        } else if (Radians.of(sim.output.get(0, 0)).lt(INTAKE_PIVOT_MIN_ANGLE)) {
            sim.setState(VecBuilder.fill(INTAKE_PIVOT_MIN_ANGLE.`in`(Radians), 0.0))
            sim.setInput(MathUtil.clamp(pivotVout.`in`(Volts), 0.0, 12.0))
        } else {
            sim.setInput(pivotVout.`in`(Volts))
        }

        input.pivotAngle = Radians.of(sim.output.get(0, 0))
        input.pivotVelocity = RadiansPerSecond.of(sim.output.get(1, 0))

        input.pivotGoal = pivotGoal
        input.pivotSetpointPos = Units.Radians.of(pivotPID.setpoint.position)
        input.pivotSetpointVel = Units.RadiansPerSecond.of(pivotPID.setpoint.velocity)
        input.pivotAtSetpoint = pivotPID.atSetpoint()

        input.pivotVoltageOut = pivotVout

        input.pivotMotorConnected = true
        input.pivotEncoderConnected = true
        input.pivotOpenLoop = pivotOpenLoop

        // run maple sim intake
        if (input.pivotAngle.lte(Degrees.of(10.0))
            && input.rollerVoltageOut.gte(Volts.of(3.0))
        ) {
            mapleSimIntake.startIntake()
        } else {
            mapleSimIntake.stopIntake()
        }
        Logger.recordOutput("Sim/held fuel", mapleSimIntake.getGamePiecesAmount())
    }

    override fun setRollerVoltage(voltage: Voltage) {
        rollerVout = voltage
    }

    override fun setPivotGoal(goal: Angle) {
        pivotOpenLoop = false
        pivotGoal = goal
    }

    override fun setPivotVoltage(voltage: Voltage) {
        pivotOpenLoop = true
        pivotVout = voltage
    }
}
