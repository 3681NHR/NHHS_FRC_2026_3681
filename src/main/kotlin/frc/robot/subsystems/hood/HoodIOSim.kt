package frc.robot.subsystems.hood

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Degree
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Kelvin
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.LinearSystemSim
import frc.robot.constants.HoodConstants.HOOD_HOME_ON_START
import frc.robot.constants.HoodConstants.HOOD_ID_GAINS
import frc.robot.constants.HoodConstants.HOOD_MAX_ANGLE
import frc.robot.constants.HoodConstants.HOOD_MIN_ANGLE
import frc.robot.constants.HoodConstants.HOOD_PID_GAINS
import frc.robot.constants.HoodConstants.HOOD_FF_GAINS
import frc.robot.constants.HoodConstants.HOOD_SETPOINT_TOLERANCE
import frc.utils.controlWrappers.ProfiledPID
import frc.utils.controlWrappers.SimpleFF

class HoodIOSim : HoodIO {

    private var homed: Boolean = HOOD_HOME_ON_START
    private var openloop: Boolean = false

    private var goal: Angle = Radians.of(0.0)
    private var vout: Voltage = Volts.of(0.0)

    private val pid = ProfiledPID(HOOD_PID_GAINS)
    private val ff = SimpleFF(HOOD_FF_GAINS)

    private val model: LinearSystem<N2, N1, N2> = LinearSystemId.identifyPositionSystem(
        HOOD_ID_GAINS.kV / (Math.PI * 2),
        HOOD_ID_GAINS.kA / (Math.PI * 2)
    )
    private val sim = LinearSystemSim<N2, N1, N2>(model, 0.0, 0.0)

    var encoderAngle: Angle = Degrees.of(0.0)
    var encoderOffset: Angle = Degrees.of(0.0)

    init {
        sim.setState(VecBuilder.fill(Degree.of(35.0).`in`(Radians), 0.0))
        encoderOffset = Radians.of(-sim.output.get(0, 0))

        pid.setTolerance(HOOD_SETPOINT_TOLERANCE.`in`(Rotations))
    }

    override fun updateInputs(input: HoodIO.HoodIOInputs) {
        sim.update(0.02)
        encoderAngle = Radians.of(sim.output.get(0, 0)).plus(encoderOffset)

        if (!openloop && homed) {
            vout = Volts.of(pid.calculate(encoderAngle.`in`(Rotations), goal.`in`(Rotations)))
            vout = vout.plus(Volts.of(ff.calculate(pid.setpoint.velocity)))
        }

        if (homed) {
            // soft limit - cant use internal, as it cant be configured while enabled
            if (encoderAngle.`in`(Rotations) >= HOOD_MAX_ANGLE.`in`(Rotations) && vout.`in`(Volts) > 0) {
                vout = Volts.of(0.0)
            }
            if (encoderAngle.`in`(Rotations) <= HOOD_MIN_ANGLE.`in`(Rotations) && vout.`in`(Volts) < 0) {
                vout = Volts.of(0.0)
            }
        }

        if (Radians.of(sim.output.get(0, 0)).gt(HOOD_MAX_ANGLE)) {
            sim.setState(VecBuilder.fill(HOOD_MAX_ANGLE.`in`(Radians), 0.0))
            sim.setInput(MathUtil.clamp(vout.`in`(Volts), -12.0, 0.0))
        } else if (Radians.of(sim.output.get(0, 0)).lt(HOOD_MIN_ANGLE)) {
            sim.setState(VecBuilder.fill(HOOD_MIN_ANGLE.`in`(Radians), 0.0))
            sim.setInput(MathUtil.clamp(vout.`in`(Volts), 0.0, 12.0))
        } else {
            sim.setInput(vout.`in`(Volts))
        }

        input.angle = encoderAngle
        input.velocity = RadiansPerSecond.of(sim.output.get(1, 0))

        input.atSetpoint = pid.atGoal()

        input.vout = vout
        input.current = Amps.of(-1.0)
        input.temp = Kelvin.of(-1.0)

        input.homed = homed
        input.openloop = openloop

        input.goal = goal
        input.setpointPos = Rotations.of(pid.setpoint.position)
    }

    override fun setGoal(goal: Angle) {
        openloop = false
        this.goal = goal
    }

    override fun setVout(vout: Voltage) {
        openloop = true
        this.vout = vout
    }

    override fun setPos(pos: Angle) {
        encoderOffset = encoderAngle.minus(encoderOffset).unaryMinus().plus(pos)
        encoderAngle = Radians.of(sim.output.get(0, 0)).plus(encoderOffset)
        pid.reset(encoderAngle.`in`(Rotations))
    }

    override fun setHomed(homed: Boolean) {
        this.homed = homed
    }

    override fun reset() {
        pid.reset(encoderAngle.`in`(Rotations))
    }
}
