package frc.robot.oldautos

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StringArraySubscriber
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.utils.AllianceUtility
import frc.utils.ExtraMath
import frc.utils.LoggedField2d
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import java.lang.reflect.Field

/**
 * A [edu.wpi.first.wpilibj.smartdashboard.SendableChooser] for selecting an autonomous program.
 *
 * How to add a new autonomous program.
 *     1.
 *     2. Add a new method in [AutoFactory] that returns a [Command].
 *     3. Add a new [AutoProgram] to [AUTO_PROGRAMS].
 *     4. Implement the autonomous program factory method.
 *     5. Test, test, and test some more.
 *
 * thanks to 2910 for 3/8 of this code
 */
@Suppress("UNCHECKED_CAST")
class AutoChooser(container: RobotContainer) {

    private val factory: AutoFactory = AutoFactory(container)
    private val robotContainer: RobotContainer = container

    private val field: LoggedField2d = LoggedField2d()
    private val chooser: LoggedDashboardChooser<AutoProgram>
    private val timeSelector: LoggedNetworkNumber = LoggedNetworkNumber("SmartDashboard/Auto/Time select", 0.0)

    private val AUTO_PROGRAMS: List<AutoProgram> = listOf(
        //            new AutoProgram("preload", AutoFactory::createPreloadAuto),
        //            //depot autos
        //            new AutoProgram("trench depot + AI", AutoFactory::createDepotAIAuto),
        //            new AutoProgram("bump depot + AI", AutoFactory::createBumpDepotAIAuto),
        //            //middle autos
        //            new AutoProgram("Left insanity", AutoFactory::createLeftPassAuto),
        //            new AutoProgram("Right not very insanity", AutoFactory::createRightPassAuto),
        //            //ai only autos
        //            new AutoProgram("AI right mid", AutoFactory::createRightAIMidAuto),
        //            new AutoProgram("AI left mid", AutoFactory::createLeftAIMidAuto),
        //            new AutoProgram("AI right zone", AutoFactory::createRightAIZoneAuto),
        //            new AutoProgram("AI left zone", AutoFactory::createLeftAIZoneAuto)
    )

    init {
        chooser = LoggedDashboardChooser("Auto Program")
        @Suppress("UNCHECKED_CAST")
        chooser.addDefaultOption("none", null as AutoProgram?)

        for (autoProgram in AUTO_PROGRAMS) {
            chooser.addOption(autoProgram.getLabel(), autoProgram)
        }

        for (program in AUTO_PROGRAMS) {
            program.update(factory)
        }

        SmartDashboard.putData("Auto/Path", field)
    }

    fun getSelected(): Command {
        return chooser.get()!!.getCommand(factory)
    }

    fun update() {
        getAlerts()
        if (!DriverStation.isEnabled() && chooser.get() != null) {
            val time = timeSelector.get() * chooser.get()!!.getPathLength(factory)

            field.getObject("trajectory").setPoses(chooser.get()!!.getPoses(factory).toList())
            field.setRobotPose(AllianceUtility.flipPose(robotContainer.getDrive().pose))
            field.getObject("start pose").setPose(chooser.get()!!.getStartingPose(factory))
            field.getObject("selected pose").setPose(chooser.get()!!.getPoseAtTime(factory, time))

            Logger.recordOutput("Auto/Selected time", ExtraMath.roundToPoint(time, 3))
            Logger.recordOutput("Auto/Total time", chooser.get()!!.getPathLength(factory))

            Logger.recordOutput(
                "Auto/Checklist/Robot in position",
                ExtraMath.poseWithinTolerance(robotContainer.getDrive().pose, chooser.get()!!.getStartingPose(factory), 0.5, Math.toRadians(20.0))
            )
        }
        Logger.recordOutput("Auto/Checklist/Auto selected", chooser.get() != null)
        Logger.recordOutput("Auto/Checklist/FMS connected", DriverStation.isFMSAttached())
        Logger.recordOutput("Auto/Checklist/Joysticks connected", DriverStation.isJoystickConnected(0) && DriverStation.isJoystickConnected(1))
        Logger.recordOutput(
            "Auto/Checklist/No alerts",
            errors.isEmpty() && warnings.isEmpty() && infos.isEmpty()
        )
        Logger.recordOutput("Auto/Checklist/Auto set", "Set correct auto program")
        Logger.recordOutput("Auto/Checklist/Odometry correct", "Confirm odometry is same with real position")
        Logger.recordOutput("Auto/Checklist/Controller ports", "Confirm controllers are connected to right ports")
        Logger.recordOutput("Auto/Checklist/DS secure", "Confirm DS is securely attached to shelf")
        Logger.recordOutput("Auto/Checklist/Gamepieces loaded", "confirm 8 preload fuel")
    }

    companion object {
        private var groups: Map<String, Any>? = null
        private val errorSubscribers: MutableMap<String, StringArraySubscriber> = HashMap()
        private val warningSubscribers: MutableMap<String, StringArraySubscriber> = HashMap()
        private val infoSubscribers: MutableMap<String, StringArraySubscriber> = HashMap()

        private var errors: Array<String> = arrayOf()
        private var warnings: Array<String> = arrayOf()
        private var infos: Array<String> = arrayOf()

        init {
            try {
                val sendableAlertsClass = Class.forName("edu.wpi.first.wpilibj.Alert\$SendableAlerts")
                val groupsField: Field = sendableAlertsClass.getDeclaredField("groups")
                groupsField.isAccessible = true
                groups = groupsField.get(null) as Map<String, Any>
            } catch (e: ClassNotFoundException) {
                e.printStackTrace()
            } catch (e: IllegalArgumentException) {
                e.printStackTrace()
            } catch (e: IllegalAccessException) {
                e.printStackTrace()
            } catch (e: NoSuchFieldException) {
                e.printStackTrace()
            } catch (e: SecurityException) {
                e.printStackTrace()
            }
        }

        /**
         * Log the current state of all alerts as outputs.
         */
        @JvmStatic
        fun getAlerts() {
            val g = groups ?: return
            for (group in g.keys) {
                if (!errorSubscribers.containsKey(group)) {
                    errorSubscribers[group] = NetworkTableInstance.getDefault()
                        .getStringArrayTopic("/SmartDashboard/$group/errors")
                        .subscribe(arrayOf())
                }
                if (!warningSubscribers.containsKey(group)) {
                    warningSubscribers[group] = NetworkTableInstance.getDefault()
                        .getStringArrayTopic("/SmartDashboard/$group/warnings")
                        .subscribe(arrayOf())
                }
                if (!infoSubscribers.containsKey(group)) {
                    infoSubscribers[group] = NetworkTableInstance.getDefault()
                        .getStringArrayTopic("/SmartDashboard/$group/infos")
                        .subscribe(arrayOf())
                }

                errors = errorSubscribers[group]!!.get()
                warnings = warningSubscribers[group]!!.get()
                infos = infoSubscribers[group]!!.get()
            }
        }
    }
}
