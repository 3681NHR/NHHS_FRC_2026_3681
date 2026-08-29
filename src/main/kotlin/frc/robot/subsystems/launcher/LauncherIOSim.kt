package frc.robot.subsystems.launcher

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.simulation.LinearSystemSim
import frc.robot.constants.LauncherConstants.LAUNCHER_FF_GAINS
import frc.robot.constants.LauncherConstants.LAUNCHER_ID_GAINS
import frc.robot.constants.LauncherConstants.LAUNCHER_PID_GAINS
import frc.robot.constants.LauncherConstants.LAUNCHER_SETPOINT_TOLERANCE
import frc.utils.controlWrappers.PID
import frc.utils.controlWrappers.SimpleFF

class LauncherIOSim : LauncherIO {

    private var goal: AngularVelocity = RPM.of(0.0)
    private var vout: Voltage = Volts.of(0.0)
    private var speed: AngularVelocity = RPM.of(0.0)
    private var openLoop: Boolean = false

    private val pid = PID(LAUNCHER_PID_GAINS)
    private val ff = SimpleFF(LAUNCHER_FF_GAINS)

    private val model: LinearSystem<N2, N1, N2> = LinearSystemId.identifyPositionSystem(
        LAUNCHER_ID_GAINS.kV / (2 * Math.PI),
        LAUNCHER_ID_GAINS.kA / (2 * Math.PI)
    )
    private val sim = LinearSystemSim<N2, N1, N2>(model, 0.01, 0.1)

    override fun updateInputs(input: LauncherIO.LauncherIOInputs) {
        sim.update(0.02)
        speed = RadiansPerSecond.of(sim.output.get(1, 0))

        if (!openLoop) {
            vout = Volts.of(pid.calculate(speed.`in`(RotationsPerSecond), goal.`in`(RotationsPerSecond)))
            vout = vout.plus(Volts.of(ff.calculate(goal.`in`(RotationsPerSecond))))
        }
        if (DriverStation.isEnabled()) {
            sim.setInput(vout.`in`(Volts) - Math.min(LAUNCHER_ID_GAINS.kS, Math.abs(vout.`in`(Volts))) * Math.signum(sim.output.get(1, 0)))
        } else {
            sim.setInput(0.0)
        }

        input.angle = Radians.of(sim.output.get(0, 0))
        input.speed = speed

        input.motorVoltageOut = vout

        input.goal = goal
        input.atSetpoint = MathUtil.isNear(goal.`in`(RPM), speed.`in`(RPM), LAUNCHER_SETPOINT_TOLERANCE.`in`(RPM))

        input.openLoop = openLoop
    }

    override fun setGoal(goal: AngularVelocity) {
        openLoop = false
        this.goal = goal
    }

    override fun setVout(vout: Voltage) {
        openLoop = true
        this.vout = vout
        this.goal = RPM.of(0.0)
    }
}
