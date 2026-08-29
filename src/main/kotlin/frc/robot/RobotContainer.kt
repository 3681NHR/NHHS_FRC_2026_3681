/* I wrote this robot code with furry paws on. Just thought I would mention that. -yarden*/

package frc.robot

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Microseconds
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import frc.robot.autos.AutoGenerator
import frc.robot.commands.SwerveWheelCharacterization
import frc.robot.constants.Constants
import frc.robot.constants.Constants.OperatorConstants
import frc.robot.constants.Constants.RobotMode
import frc.robot.constants.DriveConstants
import frc.robot.constants.FuelVisionConstants
import frc.robot.constants.HoodConstants.HOOD_MIN_ANGLE
import frc.robot.constants.IntakeConstants.INTAKE_OFFSET
import frc.robot.constants.TurretConstants.BLUE_HUB
import frc.robot.constants.TurretConstants.BLUE_PASS
import frc.robot.constants.TurretConstants.HOOD_TO_TURRET_OFFSET
import frc.robot.constants.TurretConstants.HUB_RADIUS
import frc.robot.constants.TurretConstants.PASS_RADIUS
import frc.robot.constants.TurretConstants.RED_HUB
import frc.robot.constants.TurretConstants.RED_PASS
import frc.robot.constants.TurretConstants.TURRET_OFFSET
import frc.robot.constants.VisionConstants
import frc.robot.oldautos.AutoChooser
import frc.robot.subsystems.LaunchLUT
import frc.robot.subsystems.Led
import frc.robot.subsystems.SOTMSolver
import frc.robot.subsystems.SimFuelManager
import frc.robot.subsystems.climber.Climber
import frc.robot.subsystems.climber.ClimberIO
import frc.robot.subsystems.climber.ClimberIOSim
import frc.robot.subsystems.fuelVision.FuelVision
import frc.robot.subsystems.fuelVision.FuelVisionIO
import frc.robot.subsystems.fuelVision.FuelVisionIOPhoton
import frc.robot.subsystems.fuelVision.FuelVisionIOPhotonSim
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.hood.HoodIO
import frc.robot.subsystems.hood.HoodIOReal
import frc.robot.subsystems.hood.HoodIOSim
import frc.robot.subsystems.indexer.Indexer
import frc.robot.subsystems.indexer.IndexerIO
import frc.robot.subsystems.indexer.IndexerIOReal
import frc.robot.subsystems.indexer.IndexerIOSim
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.intake.IntakeIO
import frc.robot.subsystems.intake.IntakeIOReal
import frc.robot.subsystems.intake.IntakeIOSim
import frc.robot.subsystems.kicker.Kicker
import frc.robot.subsystems.kicker.KickerIO
import frc.robot.subsystems.kicker.KickerIOReal
import frc.robot.subsystems.kicker.KickerIOSim
import frc.robot.subsystems.launcher.Launcher
import frc.robot.subsystems.launcher.LauncherIO
import frc.robot.subsystems.launcher.LauncherIOReal
import frc.robot.subsystems.launcher.LauncherIOSim
import frc.robot.subsystems.physButtons.ButtonIO
import frc.robot.subsystems.physButtons.ButtonIODIO
import frc.robot.subsystems.physButtons.ButtonIOSim
import frc.robot.subsystems.physButtons.Buttons
import frc.robot.subsystems.swerve.Drive
import frc.robot.subsystems.swerve.gyro.GyroIO
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2
import frc.robot.subsystems.swerve.gyro.GyroIOSim
import frc.robot.subsystems.swerve.module.ModuleIO
import frc.robot.subsystems.swerve.module.ModuleIOCrackingSpark
import frc.robot.subsystems.swerve.module.ModuleIOSim
import frc.robot.subsystems.turret.Turret
import frc.robot.subsystems.turret.TurretIO
import frc.robot.subsystems.turret.TurretIOReal
import frc.robot.subsystems.turret.TurretIOSim
import frc.robot.subsystems.vision.CameraIO
import frc.robot.subsystems.vision.CameraIOPhoton
import frc.robot.subsystems.vision.CameraIOPhotonSim
import frc.robot.subsystems.vision.Vision
import frc.utils.AllianceUtility
import frc.utils.BatteryVoltageSim
import frc.utils.ExtraMath
import frc.utils.HiddenConditionalCommand
import frc.utils.Joystick
import frc.utils.Joystick.DuelJoystickAxis
import frc.utils.PIDTuner
import frc.utils.Periodic
import frc.utils.ShiftTracker
import frc.utils.ZoneManager
import frc.utils.rumble.RumbleHandler
import frc.utils.rumble.RumblePreset
import frc.utils.ControllerMap.A
import frc.utils.ControllerMap.B
import frc.utils.ControllerMap.DOWN
import frc.utils.ControllerMap.LB
import frc.utils.ControllerMap.LEFT
import frc.utils.ControllerMap.LEFT_STICK_BUTTON
import frc.utils.ControllerMap.LEFT_STICK_X
import frc.utils.ControllerMap.LEFT_STICK_Y
import frc.utils.ControllerMap.LEFT_TRIGGER
import frc.utils.ControllerMap.LOGO_LEFT
import frc.utils.ControllerMap.LOGO_RIGHT
import frc.utils.ControllerMap.RB
import frc.utils.ControllerMap.RIGHT
import frc.utils.ControllerMap.RIGHT_STICK_X
import frc.utils.ControllerMap.RIGHT_STICK_Y
import frc.utils.ControllerMap.RIGHT_TRIGGER
import frc.utils.ControllerMap.UP
import frc.utils.ControllerMap.X
import frc.utils.ControllerMap.Y
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.StandardOpenOption
import java.util.Arrays
import java.util.Random
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.COTS
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig
import org.ironmaple.simulation.gamepieces.GamePieceProjectile
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

@Suppress("DEPRECATION")
class RobotContainer {

    private var currentStartTimestamp: Double = 0.0
    private var hoodSetpoint: Angle = Degrees.zero()
    private var launcherSetpoint: AngularVelocity = RPM.zero()
    private var distanceToHub: Distance = Meters.zero()

    private val rand = Random()
    private var gauss: Double = 0.4

    private var isLutInProgress: Boolean = false
    private val sysidChooser = LoggedDashboardChooser<Command>("sysid auto chooser")

    private var driveTrainSimulationConfig: DriveTrainSimulationConfig? = null
    private var driveSim: SwerveDriveSimulation? = null
    private lateinit var drive: Drive
    private lateinit var vision: Vision
    @Suppress("unused")
    private var fuelVision: FuelVision? = null
    private lateinit var turret: Turret
    private lateinit var launcher: Launcher
    private lateinit var climber: Climber
    private lateinit var hood: Hood
    private lateinit var intake: Intake
    private lateinit var kicker: Kicker
    private lateinit var buttons: Buttons
    private lateinit var indexer: Indexer

    @Suppress("unused")
    private lateinit var led: Led
    private var manual: Boolean = true

    var hubTrack: Boolean = false

    private val driverController = XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT)
    private val operatorController = XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT)

    private val resetOdometry = LoggedNetworkBoolean("Debug/Reset Odometry", false)
    private val TStop = LoggedNetworkBoolean("Overrides/Paralyze turret", false)
    private val autoTrench = LoggedNetworkBoolean("Overrides/use autotrench", true)
    private val shiftLock = LoggedNetworkBoolean("Overrides/use shift tracker", true)
    private lateinit var autoChooser: AutoChooser

    private val rumbler = RumbleHandler(driverController)
    private val opRumbler = RumbleHandler(operatorController)

    private var target: Translation2d = Translation2d()

    var apriltagLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)

    private val driverDisconnected = Alert("Driver controller disconnected (port 0).", AlertType.kWarning)
    private val operatorDisconnected = Alert("Operator controller disconnected (port 1).", AlertType.kWarning)

    private val useLead = LoggedNetworkBoolean("Overrides/Enable SOTM", true)
    private val forceFeed = LoggedNetworkBoolean("Overrides/Force Feed", false)

    private val manHoodDegrees = LoggedNetworkNumber("Manual/Hood angle degrees", HOOD_MIN_ANGLE.`in`(Degrees))
    private val manShooterRPM = LoggedNetworkNumber("Manual/Shooter speed RPM", 0.0)
    private val manTurretDegrees = LoggedNetworkNumber("Manual/Turret angle degrees", 0.0)

    private val readyDebounce = Debouncer(0.2)

    private lateinit var driverSticks: DuelJoystickAxis

    private val generator: AutoGenerator

    init {
        frc.robot.autos.Path.container = this
        generator = AutoGenerator(this)
        try {
            // load test field layout for camera offset calculation, do not use otherwise
            // e = AprilTagFieldLayout(Filesystem.getDeployDirectory().absolutePath + "/test_field.json")
        } catch (ex: Exception) {
        }

        Logger.recordOutput("AScope/zeroPose", Pose3d())

        // we use our own warnings for joysticks
        DriverStation.silenceJoystickConnectionWarning(true)

        if (RobotBase.isSimulation()) {
            // maplesim setup
            driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
                .withGyro(COTS.ofPigeon2())
                .withSwerveModule(
                    COTS.ofMark4i(
                        DCMotor.getKrakenX60(1),
                        DCMotor.getNEO(1),
                        COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                        2
                    )
                )
                .withTrackLengthTrackWidth(DriveConstants.LENGTH, DriveConstants.WIDTH)
                .withBumperSize(Inches.of(31.0), Inches.of(33.0))

            driveSim = SwerveDriveSimulation(driveTrainSimulationConfig, Constants.STARTING_POSE)
            SimulatedArena.overrideInstance(Arena2026Rebuilt(false))
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSim)
            //FIXME: this line is why your sim sucks
            //    (SimulatedArena.getInstance() as Arena2026Rebuilt).isEfficiencyMode = false
        }

        // process driver controls(radial deadzone, curve, trigger slowdown, and inversion)
        driverSticks = DuelJoystickAxis(
            { ExtraMath.processInput(
                Joystick.deadzone(OperatorConstants.LEFT_DEADBAND,
                    driverController.getRawAxis(LEFT_STICK_X), driverController.getRawAxis(LEFT_STICK_Y)
                ).x,
                -1.0,
                OperatorConstants.TRANSLATION_CURVE, 0.0) },
            { ExtraMath.processInput(
                Joystick.deadzone(OperatorConstants.LEFT_DEADBAND,
                    driverController.getRawAxis(LEFT_STICK_X), driverController.getRawAxis(LEFT_STICK_Y)
                ).y,
                -1.0,
                OperatorConstants.TRANSLATION_CURVE, 0.0) },
            { ExtraMath.processInput(
                Joystick.deadzone(OperatorConstants.RIGHT_DEADBAND,
                    driverController.getRawAxis(RIGHT_STICK_X), driverController.getRawAxis(RIGHT_STICK_Y)
                ).x,
                -0.75,
                OperatorConstants.ROTATION_CURVE, 0.0) },
            { ExtraMath.processInput(
                Joystick.deadzone(OperatorConstants.RIGHT_DEADBAND,
                    driverController.getRawAxis(RIGHT_STICK_X), driverController.getRawAxis(RIGHT_STICK_Y)
                ).y,
                -1.0,
                OperatorConstants.ROTATION_CURVE, 0.0) }
        )

        when (Constants.MODE) {
            RobotMode.REAL -> {
                // Real robot, instantiate hardware IO implementations
                vision = Vision(
                    apriltagLayout,
                    CameraIOPhoton(apriltagLayout, VisionConstants.CAMERA_CONFIGS[0]),
                    CameraIOPhoton(apriltagLayout, VisionConstants.CAMERA_CONFIGS[1]),
                    CameraIOPhoton(apriltagLayout, VisionConstants.CAMERA_CONFIGS[2]),
                    CameraIOPhoton(apriltagLayout, VisionConstants.CAMERA_CONFIGS[3])
                )
                drive = Drive(
                    GyroIOPigeon2(),
                    ModuleIOCrackingSpark(0),
                    ModuleIOCrackingSpark(1),
                    ModuleIOCrackingSpark(2),
                    ModuleIOCrackingSpark(3),
                    vision,
                    driverSticks
                )
                SOTMSolver.getInstance().setDrive(drive)
                SOTMSolver.getInstance().calculate()

                fuelVision = FuelVision(FuelVisionIOPhoton(FuelVisionConstants.CAMERA_CONFIG), drive::getPose)

                turret = Turret(TurretIOReal(), drive)
                intake = Intake(IntakeIOReal())
                launcher = Launcher(LauncherIOReal())
                hood = Hood(HoodIOReal())
                climber = Climber(object : ClimberIO {})
                kicker = Kicker(KickerIOReal())
                buttons = Buttons(ButtonIODIO(0))
                indexer = Indexer(IndexerIOReal())
            }

            RobotMode.SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                vision = Vision(
                    apriltagLayout,
                    CameraIOPhotonSim(apriltagLayout, VisionConstants.CAMERA_CONFIGS[0], driveSim!!::getSimulatedDriveTrainPose),
                    CameraIOPhotonSim(apriltagLayout, VisionConstants.CAMERA_CONFIGS[1], driveSim!!::getSimulatedDriveTrainPose),
                    CameraIOPhotonSim(apriltagLayout, VisionConstants.CAMERA_CONFIGS[2], driveSim!!::getSimulatedDriveTrainPose),
                    CameraIOPhotonSim(apriltagLayout, VisionConstants.CAMERA_CONFIGS[3], driveSim!!::getSimulatedDriveTrainPose)
                )
                if (driveSim != null) {
                    drive = Drive(
                        GyroIOSim(driveSim!!.gyroSimulation),
                        ModuleIOSim(driveSim!!.modules[0]),
                        ModuleIOSim(driveSim!!.modules[1]),
                        ModuleIOSim(driveSim!!.modules[2]),
                        ModuleIOSim(driveSim!!.modules[3]),
                        vision,
                        driverSticks
                    )
                    SOTMSolver.getInstance().setDrive(drive)
                    SOTMSolver.getInstance().calculate()

                    fuelVision = FuelVision(FuelVisionIOPhotonSim(FuelVisionConstants.CAMERA_CONFIG, driveSim!!::getSimulatedDriveTrainPose), drive::getPose)
                    intake = Intake(IntakeIOSim(driveSim!!))
                    turret = Turret(TurretIOSim(), drive)
                }
                launcher = Launcher(LauncherIOSim())
                hood = Hood(HoodIOSim())
                climber = Climber(ClimberIOSim())
                kicker = Kicker(KickerIOSim())
                buttons = Buttons(ButtonIOSim { false })
                indexer = Indexer(IndexerIOSim())
            }

            else -> {
                // Replayed robot, disable IO implementations for replay
                vision = Vision(
                    apriltagLayout,
                    object : CameraIO {},
                    object : CameraIO {},
                    object : CameraIO {},
                    object : CameraIO {}
                )
                drive = Drive(
                    object : GyroIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    vision,
                    driverSticks
                )
                SOTMSolver.getInstance().setDrive(drive)
                SOTMSolver.getInstance().calculate()

                fuelVision = FuelVision(object : FuelVisionIO {}, drive::getPose)
                intake = Intake(object : IntakeIO {})
                turret = Turret(object : TurretIO {}, drive)

                launcher = Launcher(object : LauncherIO {})
                hood = Hood(object : HoodIO {})
                climber = Climber(object : ClimberIO {})
                kicker = Kicker(object : KickerIO {})
                buttons = Buttons(object : ButtonIO {})
                indexer = Indexer(object : IndexerIO {})
            }
        }
        led = Led(launcher, hood, turret, drive) { manual }

        configureBindings()

        autoChooser = AutoChooser(this)
        sysidChooser.addDefaultOption("none", null)
        sysidChooser.addOption("drive sysid quasistatic forward", drive.sysIdQuasistatic(Direction.kForward))
        sysidChooser.addOption("drive sysid quasistatic reverse", drive.sysIdQuasistatic(Direction.kReverse))
        sysidChooser.addOption("drive sysid dynamic forward", drive.sysIdDynamic(Direction.kForward))
        sysidChooser.addOption("drive sysid dynamic reverse", drive.sysIdDynamic(Direction.kReverse))

        sysidChooser.addOption("steer sysid quasistatic forward", drive.steerSysIdQuasistatic(Direction.kForward))
        sysidChooser.addOption("steer sysid quasistatic reverse", drive.steerSysIdQuasistatic(Direction.kReverse))
        sysidChooser.addOption("steer sysid dynamic forward", drive.steerSysIdDynamic(Direction.kForward))
        sysidChooser.addOption("steer sysid dynamic reverse", drive.steerSysIdDynamic(Direction.kReverse))

        sysidChooser.addOption("angle sysid quasistatic forward", drive.angleSysIdQuasistatic(Direction.kForward))
        sysidChooser.addOption("angle sysid quasistatic reverse", drive.angleSysIdQuasistatic(Direction.kReverse))
        sysidChooser.addOption("angle sysid dynamic forward", drive.angleSysIdDynamic(Direction.kForward))
        sysidChooser.addOption("angle sysid dynamic reverse", drive.angleSysIdDynamic(Direction.kReverse))
        sysidChooser.addOption("swerve wheel radius char", SwerveWheelCharacterization(drive))
        sysidChooser.addOption("turret sysid quasistatic forward", turret.sysidQuasistatic(false))
        sysidChooser.addOption("turret sysid quasistatic reverse", turret.sysidQuasistatic(true))
        sysidChooser.addOption("turret sysid dynamic forward", turret.sysidDynamic(false))
        sysidChooser.addOption("turret sysid dynamic reverse", turret.sysidDynamic(true))

        sysidChooser.addOption("launcher sysid quasistatic forward", launcher.sysidQuasistatic(false))
        sysidChooser.addOption("launcher sysid quasistatic reverse", launcher.sysidQuasistatic(true))
        sysidChooser.addOption("launcher sysid dynamic forward", launcher.sysidDynamic(false))
        sysidChooser.addOption("launcher sysid dynamic reverse", launcher.sysidDynamic(true))

        turret.defaultCommand = turret.manPos({ turret.getAngle() }, false).ignoringDisable(true)
        launcher.defaultCommand = launcher.voltageControl { Volts.of(0.0) }
        kicker.defaultCommand = kicker.hold().ignoringDisable(true)
        drive.defaultCommand = drive.teleopDrive().ignoringDisable(true)
        hood.defaultCommand = hood.instantPositionControl { HOOD_MIN_ANGLE }.ignoringDisable(true)
        climber.defaultCommand = climber.voltageControl { Volts.zero() }.ignoringDisable(true)
        indexer.defaultCommand = indexer.stop().ignoringDisable(true)
    }

    private fun configureBindings() {
        Trigger { driverController.getRawButton(11) && Constants.MODE == RobotMode.SIM }.onTrue(InstantCommand({
            (SimulatedArena.getInstance() as Arena2026Rebuilt).outpostDump(AllianceUtility.getAlliance() == Alliance.Blue)
        }))

        Trigger(TStop).whileTrue(
            turret.stop().repeatedly().ignoringDisable(true).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        )

        // reset odometry dashboard button
        resetOdometry.set(false)
        Trigger(resetOdometry::get).onTrue(InstantCommand({
            resetOdometry.set(false)
            drive.setPose(Constants.STARTING_POSE)
        }))

        // send haptic command when 5 seconds are left in shift
        Trigger { ShiftTracker.getTimeLeftInShift() < 5 }.onTrue(InstantCommand({
            rumbler.overrideQue(RumblePreset.TAP.load())
            opRumbler.overrideQue(RumblePreset.TAP.load())
        }))

        // move wheels to X, makes robot hard to push
        Trigger { driverController.getRawButton(LOGO_RIGHT) }.whileTrue(InstantCommand({
            drive.stopWithX()
        }, drive).repeatedly())

        // reset gyro angle
        Trigger { driverController.getRawButton(LOGO_LEFT) }.onTrue(InstantCommand({
            drive.resetGyro(if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) Math.PI else 0.0)
            rumbler.overrideQue(RumblePreset.TAP.load())
        }))

        // toggle field oriented driving
        Trigger { driverController.getRawButton(LEFT_STICK_BUTTON) }.onTrue(InstantCommand({
            drive.setFOD(!drive.getFOD())
            rumbler.overrideQue(RumblePreset.TAP.load())
        }))
        Trigger { driverController.getRawAxis(RIGHT_TRIGGER) > 0.2 }.whileTrue(hood.go())
        Trigger { driverController.getRawAxis(RIGHT_TRIGGER) > 0.7 }
            .whileTrue(fire())
        // intake :3
        Trigger { driverController.getRawAxis(LEFT_TRIGGER) > 0.5 }
            .whileTrue(intake.intake())

        // force teleop drive
        Trigger { driverController.pov == UP }.onTrue(drive.teleopDrive())

        Trigger { driverController.getRawButton(X) }.whileTrue(
            hood.positionControl { HOOD_MIN_ANGLE }.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        )

        // toggle auto track command
        Trigger { driverController.getRawButton(B) }.onTrue(
            getTrackCommand()
        )

        //set turret to preset angle mode
        Trigger { driverController.getRawButton(A) }.onTrue(
            getManShooterCommand()
        )
        Trigger { driverController.getRawButton(Y) }.onTrue(
            HiddenConditionalCommand(saveLutEntry(), startLutTimer()) { isLutInProgress }
        )
        Trigger { driverController.pov == RIGHT }.onTrue(hood.home())
        Trigger { driverController.pov == LEFT }.onTrue(climber.home())

        Trigger { driverController.getRawButton(RB) }.whileTrue(Commands.parallel(kicker.reverse(), indexer.reverse()))
        Trigger { driverController.getRawButton(LB) }.whileTrue(Commands.parallel(intake.outtake()))

        Trigger { driverController.pov == DOWN }.onTrue(climber.toggle())

        Trigger { (inTrench() && autoTrench.getAsBoolean() && !DriverStation.isAutonomous()) || driverController.getRawButton(X) }.whileTrue(
            drive.TrenchAlignDrive()
                .alongWith(hood.instantPositionControl { HOOD_MIN_ANGLE }.withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
                .withName("trench mode")
        )

        Trigger { buttons.get(0) && !DriverStation.isEnabled() }.onTrue(hood.forceHome())
    }

    fun periodic() {
        rumbler.update(Constants.EVENT_LOOP_TIME)
        ZoneManager.updateRobotPose(drive.pose)
        PIDTuner.updateTunables()
        SOTMSolver.getInstance().target = target
        driverDisconnected.set(!driverController.isConnected)
        operatorDisconnected.set(!operatorController.isConnected)

        val hub = if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) RED_HUB else BLUE_HUB
        val pass = drive.pose.translation.nearest(Arrays.asList(*if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) RED_PASS else BLUE_PASS))
        hubTrack = (if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) drive.pose.x > 12 else drive.pose.x < 4.5)
        Logger.recordOutput("Subsystems/Turret/track/tracking hub", hubTrack)
        target = if (hubTrack) hub else pass

        autoChooser.update()

        Logger.recordOutput("AScope/Components", *arrayOf(
            Pose3d(0.0, 0.0, climber.getPosition().`in`(Meters), Rotation3d()),
            Pose3d(INTAKE_OFFSET, Rotation3d(Radians.zero(), intake.getAngle(), Degrees.of(180.0))),
            Pose3d(TURRET_OFFSET, Rotation3d(0.0, 0.0, turret.getAngle().`in`(Radians))),
            Pose3d(
                TURRET_OFFSET.plus(HOOD_TO_TURRET_OFFSET.rotateBy(Rotation3d(0.0, 0.0, turret.getAngle().`in`(Radians)))),
                Rotation3d(0.0, hood.getAngle().minus(Degrees.of(25.0)).`in`(Radians), turret.getAngle().`in`(Radians))
            )
        ))
        Logger.recordOutput("target dist", Meters.of(target.getDistance(drive.pose.translation)))

        SOTMSolver.getInstance().setLUT(if (hubTrack) LaunchLUT.LUTHub else LaunchLUT.LUTPass)
        gauss = rand.nextGaussian() * 0.1 + 0.4
    }

    fun simPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic()

        Logger.recordOutput("Sim/simulatedVoltage", BatteryVoltageSim.getInstance().calculateVoltage())
        Logger.recordOutput("Sim/FieldSimulation/RobotPose", driveSim!!.simulatedDriveTrainPose)

        Logger.recordOutput("Sim/FieldSimulation/Fuel", *SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"))
    }

    fun getAutonomousCommand(): Command? {
        val auto: Command?
        if (sysidChooser.get() == null || DriverStation.isFMSAttached()) {
            auto = generator.getCommand()
        } else {
            auto = sysidChooser.get()
        }
        return auto
    }

    fun resetDrivetrain(pose: Pose2d) {
        driveSim!!.setSimulationWorldPose(pose)
    }

    fun enableTeleop() {
        CommandScheduler.getInstance().schedule(drive.teleopDrive())
        drive.setCallback()
    }

    fun enableAuto() {
        if (RobotBase.isSimulation()) {
            SimulatedArena.getInstance().resetFieldForAuto()
            //preload 8
            SimFuelManager.getInstance().intake!!.setGamePiecesCount(8)
        }
    }

    fun getDrive(): Drive = drive

    /**
     * get command for turret, shooter, and hood to track current target, lead may be disabled through the useLead var
     */
    fun getTrackCommand(): Command {
        return HiddenConditionalCommand(
            ParallelCommandGroup(
                turret.trackWithLead { if (hubTrack) HUB_RADIUS else PASS_RADIUS },
                launcher.velocityControl { SOTMSolver.getInstance().getParams(false).speed() },
                hood.positionControl { SOTMSolver.getInstance().getParams(false).hoodAngle() },
                InstantCommand({ manual = false })
            ).withName("track with lead"),

            ParallelCommandGroup(
                turret.track({ target }, { if (hubTrack) HUB_RADIUS else PASS_RADIUS }),
                launcher.velocityControl { LaunchLUT.get(Meters.of(target.getDistance(drive.pose.translation)), true, if (hubTrack) LaunchLUT.LUTHub else LaunchLUT.LUTPass).speed() },
                hood.positionControl { LaunchLUT.get(Meters.of(target.getDistance(drive.pose.translation)), true, if (hubTrack) LaunchLUT.LUTHub else LaunchLUT.LUTPass).hoodAngle() },
                InstantCommand({ manual = false })
            ).withName("track without lead"),
            useLead::get
        ).finallyDo(Runnable { manual = true })
    }

    fun getManShooterCommand(): Command {
        return ParallelCommandGroup(
            turret.manPos({ Degrees.of(manTurretDegrees.getAsDouble()) }, false),
            launcher.velocityControl { RPM.of(manShooterRPM.getAsDouble()) },
            hood.positionControl { Degrees.of(manHoodDegrees.getAsDouble()) },
            InstantCommand({ manual = true })
        ).finallyDo(Runnable { manual = false }).withName("manual targeting")
    }

    fun getSimFireCommand(): Command {
        return InstantCommand({
            if (SimFuelManager.getInstance().intake!!.obtainGamePieceFromIntake()) {
                val launchvel = launcher.getSpeed().`in`(RPM) * 2 * Math.PI * Units.inchesToMeters(2.0) / 60.0
                val angle = hood.getAngle().plus(Degrees.of(90.0)).`in`(Radians)
                val turretAngle = driveSim!!.simulatedDriveTrainPose.rotation.radians + turret.getAngle().plus(Degrees.of(180.0)).`in`(Radians)
                val fuel = GamePieceProjectile(
                    RebuiltFuelOnField.REBUILT_FUEL_INFO,
                    driveSim!!.simulatedDriveTrainPose.translation.plus(
                        Translation2d(
                            Math.cos(driveSim!!.simulatedDriveTrainPose.rotation.radians) * TURRET_OFFSET.x,
                            Math.sin(driveSim!!.simulatedDriveTrainPose.rotation.radians) * TURRET_OFFSET.x
                        )
                    ),
                    Translation2d(
                        driveSim!!.driveTrainSimulatedChassisSpeedsFieldRelative.vxMetersPerSecond,
                        driveSim!!.driveTrainSimulatedChassisSpeedsFieldRelative.vyMetersPerSecond
                    ).plus(Translation2d(Math.cos(angle) * launchvel, 0.0).rotateBy(Rotation2d(turretAngle))),
                    Units.inchesToMeters(20.0),
                    Math.sin(angle) * launchvel,
                    Rotation3d()
                )

                fuel.withTouchGroundHeight(Inches.of(3.0).`in`(Meters))
                fuel.enableBecomesGamePieceOnFieldAfterTouchGround()
                SimulatedArena.getInstance().addGamePieceProjectile(fuel)
            }
        }).andThen(WaitCommand(gauss)).repeatedly()
    }

    @AutoLogOutput
    private fun inTrench(): Boolean {
        val center = Translation2d(8.269, 4.038)
        //pos with lead
        val offsetPos = drive.pose.translation.plus(
            Translation2d(
                drive.getFieldChassisSpeeds().vxMetersPerSecond * 0.5,
                drive.getFieldChassisSpeeds().vyMetersPerSecond * 0.5
            )
        )
        //pos without lead
        val pos = drive.pose.translation

        val width = 1.75
        val height = 2.0

        val xOffset = 2.61
        val yOffset = 2.5

        return (
            (pos.x > center.x + xOffset && pos.x < center.x + xOffset + width && pos.y > center.y + yOffset && pos.y < center.y + yOffset + height) ||
                (pos.x > center.x + xOffset && pos.x < center.x + xOffset + width && pos.y < center.y - yOffset && pos.y > center.y - yOffset - height) ||
                (pos.x < center.x - xOffset && pos.x > center.x - xOffset - width && pos.y < center.y - yOffset && pos.y > center.y - yOffset - height) ||
                (pos.x < center.x - xOffset && pos.x > center.x - xOffset - width && pos.y > center.y + yOffset && pos.y < center.y + yOffset + height)
            ) ||
            (
                (offsetPos.x > center.x + xOffset && offsetPos.x < center.x + xOffset + width && offsetPos.y > center.y + yOffset && offsetPos.y < center.y + yOffset + height) ||
                    (offsetPos.x > center.x + xOffset && offsetPos.x < center.x + xOffset + width && offsetPos.y < center.y - yOffset && offsetPos.y > center.y - yOffset - height) ||
                    (offsetPos.x < center.x - xOffset && offsetPos.x > center.x - xOffset - width && offsetPos.y < center.y - yOffset && offsetPos.y > center.y - yOffset - height) ||
                    (offsetPos.x < center.x - xOffset && offsetPos.x > center.x - xOffset - width && offsetPos.y > center.y + yOffset && offsetPos.y < center.y + yOffset + height)
                )
    }

    fun fire(): Command {
        return Commands.parallel(
            hood.go(),
            HiddenConditionalCommand(
                HiddenConditionalCommand(
                    getSimFireCommand(),
                    Commands.parallel(
                        kicker.feed(),
                        indexer.feed()
                    ).withName("shoot"),
                    { Constants.MODE == RobotMode.SIM }
                ),
                Commands.run({}),
                { isReady() && (!hubTrack || canScore()) }
            )
        )
    }

    fun getFuelVision(): FuelVision = fuelVision!!

    fun startLutTimer(): Command {
        return InstantCommand({
            currentStartTimestamp = Logger.getTimestamp().toDouble()
            hoodSetpoint = hood.getSetpoint()
            launcherSetpoint = launcher.getSetpoint()
            distanceToHub = Meters.of(target.getDistance(drive.pose.translation))
        })
    }

    fun saveLutEntry(): Command {
        return InstantCommand({
            val deltaTime = Logger.getTimestamp().toDouble() - currentStartTimestamp

            val line = "new ShotParams(Meters.of(%f), Degrees.of(%f), RPM.of(%f), Microseconds.of(%f));"
                .format(
                    distanceToHub.`in`(Meters),
                    hoodSetpoint.`in`(Degrees),
                    launcherSetpoint.`in`(RPM),
                    deltaTime
                )
            Logger.recordOutput("LutEntry", line)
            try {
                Files.writeString(
                    Path.of("/U/lut.txt"),
                    line + "\n",
                    StandardOpenOption.CREATE,
                    StandardOpenOption.APPEND
                )
            } catch (e: Exception) {
                println(e.message)
                val a = Alert("saving lut failed", AlertType.kWarning)
                a.set(true)
                // kill ourselves
            }
        })
    }

    @AutoLogOutput
    fun isReady(): Boolean {
        return (readyDebounce.calculate(turret.isReady() && launcher.isReady() && hood.isReady()) || forceFeed.getAsBoolean())
    }

    @AutoLogOutput
    fun canScore(): Boolean {
        return (ShiftTracker.canScore() || !shiftLock.getAsBoolean())
    }

    fun intake(): Command {
        return intake.intake()
    }

    fun getHood(): Hood = hood

    fun unload(): Command {
        return kicker.reverse()
    }
}
