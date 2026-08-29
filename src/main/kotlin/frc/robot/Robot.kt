package frc.robot

import com.pathplanner.lib.commands.PathfindingCommand
import edu.wpi.first.net.WebServer
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.constants.Constants
import frc.utils.AllianceUtility
import frc.utils.BatteryVoltageSim
import frc.utils.Periodic
import frc.utils.ShiftTracker
import frc.utils.TimerHandler
import frc.utils.motorWrappers.SparkMax
import frc.utils.motorWrappers.TalonFX
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
class Robot : LoggedRobot() {
    private var autonomousCommand: Command? = null
    private lateinit var robotContainer: RobotContainer

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    override fun robotInit() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME)
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE)
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA)
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE)
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH)
        when (BuildConstants.DIRTY) {
            0 -> Logger.recordMetadata("GitDirty", "All changes committed")
            1 -> Logger.recordMetadata("GitDirty", "Uncomitted changes")
            else -> Logger.recordMetadata("GitDirty", "Unknown")
        }

        // Set up data receivers & replay source
        when (Constants.MODE) {
            Constants.RobotMode.REAL -> {
                // Running on a real robot, log to a USB stick ("/U/logs") and NT
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }
            Constants.RobotMode.SIM -> {
                // Running a simulator, log to NT
                // Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }
            Constants.RobotMode.REPLAY -> {
                // Replaying a log, set up replay source
                setUseTiming(false) // Run as fast as possible
                val logPath = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")))
            }
        }

        // Start AdvantageKit logger
        // initalize robot container
        // LoggedPowerDistribution.getInstance(1, ModuleType.kRev);
        Logger.start()
        robotContainer = RobotContainer()

        // start timerhandler
        TimerHandler.init()
        // start elastic dashboard remote layout downloading server
        WebServer.start(Constants.ELASTIC_LAYOUT_PORT, Filesystem.getDeployDirectory().absolutePath)
        // motor wrappers
        SparkMax.initAlerts()
        TalonFX.initAlerts()

        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand())
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing finished or
        // interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything
        // in the Command-based framework to work.
        CommandScheduler.getInstance().run()
        TimerHandler.update()

        robotContainer.periodic()
//        SparkMax.periodic();
//        TalonFX.periodic();
        ShiftTracker.update()
        Periodic.updateAll()
    }

    /** This function is called once each time the robot enters Disabled mode. */
    override fun disabledInit() {
        // Elastic.selectTab(0);
    }

    override fun disabledPeriodic() {
        AllianceUtility.update()
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * [RobotContainer] class.
     */
    override fun autonomousInit() {
        //  Elastic.selectTab(0);
        autonomousCommand = robotContainer.getAutonomousCommand()

        robotContainer.enableAuto()

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand)
        }
        if (RobotBase.isSimulation()) {
            SimulatedArena.getInstance().resetFieldForAuto()
        }
        TimerHandler.initAuto()
        ShiftTracker.start()
    }

    /** This function is called periodically during autonomous. */
    override fun autonomousPeriodic() {
        TimerHandler.updateAuto()
    }

    override fun teleopInit() {
        //  Elastic.selectTab(0);
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        TimerHandler.initTeleop()
        robotContainer.enableTeleop()

        autonomousCommand?.cancel()

        if (!ShiftTracker.isRunning()) {
            ShiftTracker.start()
        }
    }

    /** This function is called periodically during operator control. */
    override fun teleopPeriodic() {
        // m_robotContainer.Periodic();
        TimerHandler.updateTeleop()
    }

    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    /** This function is called periodically during test mode. */
    override fun testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    override fun simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    override fun simulationPeriodic() {
        robotContainer.simPeriodic()

        // update battery voltage(set as roborio input voltage)
        BatteryVoltageSim.getInstance().calculateVoltage()
    }

    override fun teleopExit() {
        ShiftTracker.reset()
    }
}
