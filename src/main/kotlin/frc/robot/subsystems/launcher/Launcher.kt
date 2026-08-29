package frc.robot.subsystems.launcher

import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.constants.LauncherConstants.LAUNCHER_SYSID_CONFIG
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier

class Launcher(
    private val io: LauncherIO
) : SubsystemBase() {

    private val inputs = LauncherIOInputsAutoLogged()

    private val sysid = SysIdRoutine(LAUNCHER_SYSID_CONFIG, SysIdRoutine.Mechanism({ v: Voltage -> io.setVout(v) }, null, this))
    private val runningSysid = Alert("Launcher sysid running", Alert.AlertType.kInfo)

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("IO/Launcher", inputs)
        Logger.recordOutput("Subsystems/Launcher/state", currentCommand?.name ?: "none")
    }

    fun velocityControl(vel: Supplier<AngularVelocity>): Command {
        return Commands.run(Runnable { io.setGoal(vel.get()) }, this).withName("Velocity Control")
    }

    fun voltageControl(volt: Supplier<Voltage>): Command {
        return Commands.run(Runnable { io.setVout(volt.get()) }, this).withName("Voltage Control")
    }

    fun sysidQuasistatic(reverse: Boolean): Command {
        return sysid.quasistatic(if (reverse) SysIdRoutine.Direction.kReverse else SysIdRoutine.Direction.kForward)
            .raceWith(Commands.run(Runnable {
                runningSysid.set(true)
                runningSysid.setText("Turret sysid running: Quasistatic, " + if (reverse) "reverse" else "forward")
            }))
            .withName("Quasistatic sysid: " + if (reverse) "reverse" else "forward")
    }

    fun sysidDynamic(reverse: Boolean): Command {
        return sysid.dynamic(if (reverse) SysIdRoutine.Direction.kReverse else SysIdRoutine.Direction.kForward)
            .raceWith(Commands.run(Runnable {
                runningSysid.set(true)
                runningSysid.setText("Turret sysid running: Dynamic, " + if (reverse) "reverse" else "forward")
            }))
            .withName("Dynamic sysid: " + if (reverse) "reverse" else "forward")
    }

    fun getSetpoint(): AngularVelocity = inputs.goal

    @AutoLogOutput(key = "Subsystems/Launcher/ready")
    fun isReady(): Boolean = inputs.atSetpoint || inputs.openLoop

    fun getSpeed(): AngularVelocity = inputs.speed
}
