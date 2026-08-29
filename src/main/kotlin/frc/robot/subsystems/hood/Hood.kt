package frc.robot.subsystems.hood

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.commands.HomeCommand
import frc.robot.constants.HoodConstants.HOOD_HOME_STOP_THRESH
import frc.robot.constants.HoodConstants.HOOD_HOME_STOP_TIME
import frc.robot.constants.HoodConstants.HOOD_HOME_VOLTAGE
import frc.robot.constants.HoodConstants.HOOD_MAX_ANGLE
import frc.robot.constants.HoodConstants.HOOD_MIN_ANGLE
import frc.utils.ExtraMath
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier

class Hood(
    private val io: HoodIO
) : SubsystemBase() {

    private val inputs = HoodIOInputsAutoLogged()

    private val notHomed = Alert("Hood not homed", Alert.AlertType.kError)

    var homing: Boolean = false

    private var goal: Angle? = null

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("IO/Hood", inputs)
        notHomed.set(!inputs.homed)

        Logger.recordOutput("Subsystems/Hood/state", currentCommand?.name ?: "none")
    }

    /**
     * position control, soft limits apply, and setpoint is clamped
     * @param pos target angle
     * @return Command that drives to pos
     */
    fun positionControl(pos: Supplier<Angle>): Command {
        return Commands.run(Runnable {
            this.goal = ExtraMath.clamp(pos.get(), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE)
        }).withName("position control")
    }

    fun instantPositionControl(pos: Supplier<Angle>): Command {
        return Commands.run(Runnable {
            goal = ExtraMath.clamp(pos.get(), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE)
            io.setGoal(goal!!)
        }, this).withName("position control (instant)")
    }

    /**
     * set openloop vout, soft limits will still apply if homed
     * @param vout - voltage to apply
     * @return Command that runs at voltage
     */
    fun voltageControl(vout: Supplier<Voltage>): Command {
        return Commands.run(Runnable {
            io.setVout(vout.get())
            this.goal = null
        }, this).withName("voltage control")
    }

    /**
     * reset angle to min value and set homed to true
     * @return Command that sets homed to true and resets position to min
     */
    fun forceHome(): Command {
        return InstantCommand(Runnable {
            io.setHomed(true)
            io.setPos(HOOD_MIN_ANGLE)
        }, this).ignoringDisable(true).withName("force home")
    }

    /**
     * uses voltage commands to auto home
     * @return Command that applies a small voltage until the hood stops,
     */
    fun home(): Command {
        val c = InstantCommand(Runnable {
            io.setHomed(false)
            homing = true
        }).andThen(
            HomeCommand(
                HOOD_HOME_VOLTAGE,
                HOOD_HOME_STOP_TIME,
                { HOOD_HOME_STOP_THRESH.gte(inputs.velocity) },
                { v -> io.setVout(v) },
                {
                    io.setPos(HOOD_MIN_ANGLE)
                    io.setHomed(true)
                    io.setGoal(HOOD_MIN_ANGLE)
                    io.reset()
                }
            )
        ).andThen(InstantCommand(Runnable { homing = false }))
        c.addRequirements(this)
        c.setName("auto home")
        return c
    }

    fun go(): Command {
        val c = Commands.run(Runnable {
            if (goal != null) {
                io.setGoal(goal!!)
            }
        })
        c.addRequirements(this)
        c.setName("move to goal")
        return c
    }

    fun getAngle(): Angle = inputs.angle

    fun getSetpoint(): Angle = inputs.goal

    @AutoLogOutput(key = "Subsystems/Hood/ready")
    fun isReady(): Boolean {
        return (this.goal != null || inputs.openloop) && inputs.homed
    }

    fun isHomed(): Boolean = inputs.homed

    fun isHoming(): Boolean = homing

    fun retract(): Command = instantPositionControl { HOOD_MIN_ANGLE }
}
