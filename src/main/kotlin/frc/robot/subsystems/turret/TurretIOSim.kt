package frc.robot.subsystems.turret

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.simulation.LinearSystemSim
import frc.robot.constants.TurretConstants.TURRET_FF_GAINS
import frc.robot.constants.TurretConstants.TURRET_ID_GAINS
import frc.robot.constants.TurretConstants.TURRET_PID_GAINS
import frc.robot.constants.TurretConstants.TURRET_SETPOINT_TOLERANCE
import frc.utils.controlWrappers.ProfiledPID
import frc.utils.controlWrappers.SimpleFF

class TurretIOSim : TurretIO {

    private var goal: Angle = Radians.of(0.0)
    private var openLoop = false
    private var Vout: Voltage = Volts.of(0.0)
    private var angle: Angle = Radians.of(0.0)

    private var tolerance: Angle = TURRET_SETPOINT_TOLERANCE

    private val pid = ProfiledPID(TURRET_PID_GAINS)
    private val ff = SimpleFF(TURRET_FF_GAINS)

    private val model: LinearSystem<N2, N1, N2> = LinearSystemId.identifyPositionSystem(TURRET_ID_GAINS.kV, TURRET_ID_GAINS.kA)
    private val sim = LinearSystemSim<N2, N1, N2>(model, 0.0, 0.0)

    override fun updateInputs(input: TurretIO.TurretIOInputs) {
        sim.update(0.02)
        angle = Radians.of(sim.output.get(0, 0))

        if (!openLoop) {
            Vout = Volts.of(pid.calculate(angle.`in`(Radians), goal.`in`(Radians)))
            Vout = Vout.plus(Volts.of(ff.calculate(pid.setpoint.velocity)))
        }
        if (DriverStation.isEnabled()) {
            sim.setInput(Vout.`in`(Volts) - Math.min(TURRET_ID_GAINS.kS, Math.abs(Vout.`in`(Volts))) * Math.signum(sim.output.get(1, 0)))
        } else {
            sim.setInput(-Math.min(TURRET_ID_GAINS.kS, Math.abs(Vout.`in`(Volts))) * Math.signum(sim.output.get(1, 0)))
        }

        input.angle = Radians.of(sim.output.get(0, 0))
        input.motorAngle = input.angle
        input.speed = RadiansPerSecond.of(sim.output.get(1, 0))

        input.motorVoltageOut = Vout

        input.goal = goal
        input.setpointPos = Radians.of(pid.setpoint.position)
        input.setpointVel = RadiansPerSecond.of(pid.setpoint.velocity)
        input.atSetpoint = MathUtil.isNear(goal.`in`(Radians), angle.`in`(Radians), tolerance.`in`(Radians))

        input.openLoop = openLoop

        input.tolerance = tolerance
    }

    override fun setGoal(goal: Angle) {
        this.openLoop = false
        this.goal = goal
    }

    override fun setVout(vout: Voltage) {
        this.openLoop = true
        Vout = vout
    }

    override fun setTolerance(tol: Angle) {
        tolerance = tol
    }
}
