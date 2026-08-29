package frc.robot.subsystems.swerve

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.IdealStartingState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.Waypoint
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.commands.FineTuneAlign
import frc.robot.commands.PlayCommand
import frc.robot.constants.Constants
import frc.robot.constants.Constants.RobotMode
import frc.robot.constants.DriveConstants.ANGULAR_VELOCITY_COEFFICIENT
import frc.robot.constants.DriveConstants.ANGLE_MAX_VELOCITY
import frc.robot.constants.DriveConstants.AUTO_ALIGN_ANGLE_MAX_OFFSET
import frc.robot.constants.DriveConstants.AUTO_ANGLE_PID
import frc.robot.constants.DriveConstants.DRIVE_SYSID_TIMEOUT
import frc.robot.constants.DriveConstants.DRIVE_SYSID_VRAMP
import frc.robot.constants.DriveConstants.DRIVE_SYSID_VSTEP
import frc.robot.constants.DriveConstants.LENGTH
import frc.robot.constants.DriveConstants.MAX_ACCEL_PP
import frc.robot.constants.DriveConstants.MAX_ANGLE_ACCEL_PP
import frc.robot.constants.DriveConstants.MAX_ANGLE_SPEED_PP
import frc.robot.constants.DriveConstants.MAX_SPEED_PP
import frc.robot.constants.DriveConstants.MODULE_POSITIONS
import frc.robot.constants.DriveConstants.PP_CONFIG
import frc.robot.constants.DriveConstants.TRANS_PID
import frc.robot.constants.DriveConstants.TRANS_PID_SIM
import frc.robot.constants.DriveConstants.TURN_SYSID_TIMEOUT
import frc.robot.constants.DriveConstants.TURN_SYSID_VRAMP
import frc.robot.constants.DriveConstants.TURN_SYSID_VSTEP
import frc.robot.constants.DriveConstants.USE_VISION
import frc.robot.constants.DriveConstants.WIDTH
import frc.robot.subsystems.swerve.gyro.GyroIO
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged
import frc.robot.subsystems.swerve.module.Module
import frc.robot.subsystems.swerve.module.ModuleIO
import frc.robot.subsystems.vision.Vision
import frc.utils.Joystick.DuelJoystickAxis
import frc.utils.LoggedField2d
import frc.utils.PhoenixOdometryThread
import frc.utils.SparkOdometryThread
import frc.utils.controlWrappers.PID
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import java.util.function.DoubleSupplier
import java.util.function.Supplier

class Drive(
    private val gyroIO: GyroIO,
    flModuleIO: ModuleIO,
    frModuleIO: ModuleIO,
    blModuleIO: ModuleIO,
    brModuleIO: ModuleIO,
    private val vision: Vision,
    private val driverSticks: DuelJoystickAxis
) : SubsystemBase() {

    private var FODEnabled: Boolean = true

    private val angleController = PID(AUTO_ANGLE_PID)
    private val vyController = PID(if (RobotBase.isReal()) TRANS_PID else TRANS_PID_SIM)

    @JvmField
    var autoController: PPHolonomicDriveController = PPHolonomicDriveController(
        PIDConstants(TRANS_PID.kP, TRANS_PID.kI, TRANS_PID.kD),
        PIDConstants(AUTO_ANGLE_PID.kP, AUTO_ANGLE_PID.kI, AUTO_ANGLE_PID.kD)
    )

    private val gyroInputs = GyroIOInputsAutoLogged()
    private val modules: Array<Module> = arrayOf(
        Module(flModuleIO, 0),
        Module(frModuleIO, 1),
        Module(blModuleIO, 2),
        Module(brModuleIO, 3)
    )
    private val driveSysId: SysIdRoutine
    private val steerSysId: SysIdRoutine
    private val angleSysId: SysIdRoutine
    private val field = LoggedField2d()
    private val gyroDisconnectedAlert = Alert("Disconnected gyro, using kinematics as fallback.", Alert.AlertType.kError)

    private val kinematics = SwerveDriveKinematics(*MODULE_POSITIONS)
    private var rawGyroRotation: Angle = Radians.of(0.0)
    private var lastModulePositions: Array<SwerveModulePosition> = arrayOf(
        SwerveModulePosition(),
        SwerveModulePosition(),
        SwerveModulePosition(),
        SwerveModulePosition()
    )
    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d(rawGyroRotation.`in`(Radians)),
        lastModulePositions,
        Constants.STARTING_POSE
    )

    private var follow: Command = InstantCommand()
    private var play: Command = PlayCommand { follow }

    init {
        angleController.enableContinuousInput(-Math.PI, Math.PI)

        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit)

        SparkOdometryThread.getInstance().start()
        PhoenixOdometryThread.getInstance().start()

        AutoBuilder.configure(
            { getPose() },
            { pose: Pose2d -> setPose(pose) },
            { getChassisSpeeds() },
            { speeds: ChassisSpeeds -> runVelocity(speeds) },
            autoController,
            PP_CONFIG,
            { DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red },
            this
        )
        PathPlannerLogging.setLogActivePathCallback { activePath ->
            Logger.recordOutput(
                "Subsystems/Swerve/Odometry/Trajectory", *activePath.toTypedArray()
            )
            field.getObject("PP/activePath").setPoses(activePath)
        }

        driveSysId = SysIdRoutine(
            SysIdRoutine.Config(
                DRIVE_SYSID_VRAMP,
                DRIVE_SYSID_VSTEP,
                DRIVE_SYSID_TIMEOUT
            ) { state -> Logger.recordOutput("Subsystems/Swerve/Drive/SysIdState", state.toString()) },
            SysIdRoutine.Mechanism({ voltage: Voltage -> runCharacterization(voltage) }, null, this)
        )
        steerSysId = SysIdRoutine(
            SysIdRoutine.Config(
                TURN_SYSID_VRAMP,
                TURN_SYSID_VSTEP,
                TURN_SYSID_TIMEOUT
            ) { state -> Logger.recordOutput("Subsystems/Swerve/Drive/SteerSysIdState", state.toString()) },
            SysIdRoutine.Mechanism({ voltage: Voltage -> runSteerCharacterization(voltage) }, null, this)
        )
        angleSysId = SysIdRoutine(
            SysIdRoutine.Config(
                null,
                null,
                Seconds.of(10.0)
            ) { state -> Logger.recordOutput("Subsystems/Swerve/AngleSysIdState", state.toString()) },
            SysIdRoutine.Mechanism({ voltage: Voltage -> runAngleCharacterization(voltage.`in`(Volts)) }, null, this)
        )

        YAGSLWidget.maxAngularVelocity = getMaxAngularSpeedRadPerSec()
        YAGSLWidget.maxSpeed = getMaxLinearSpeedMetersPerSec()
        YAGSLWidget.moduleCount = 4
        YAGSLWidget.sizeFrontBack = LENGTH.`in`(Meters)
        YAGSLWidget.sizeLeftRight = WIDTH.`in`(Meters)
        YAGSLWidget.wheelLocations = DoubleArray(8)

        angleController.setTolerance(AUTO_ALIGN_ANGLE_MAX_OFFSET.`in`(Radians))

        // Preserve exact Java behavior: original condition was `i > MODULE_POSITIONS.length` (bug, never iterates)
        var idx = 0
        while (idx > MODULE_POSITIONS.size) {
            val t = MODULE_POSITIONS[idx]
            YAGSLWidget.wheelLocations[idx * 2] = t.x
            YAGSLWidget.wheelLocations[idx * 2 + 1] = t.y
            idx += 2
        }
    }

    fun setCallback() {
        PathPlannerLogging.setLogTargetPoseCallback { targetPose ->
            Logger.recordOutput("Subsystems/Swerve/Odometry/TrajectorySetpoint", targetPose)
            field.getObject("PP/targetpose").setPoses(targetPose)
        }
    }

    override fun periodic() {
        odometryLock.lock()
        gyroIO.updateInputs(gyroInputs)
        Logger.processInputs("IO/Drive/Gyro", gyroInputs)

        val disabled = DriverStation.isDisabled()
        for (module in modules) {
            module.periodic()
            if (disabled) {
                module.stop()
            }
        }

        if (USE_VISION) {
            val estimates = vision.getPose()
            if (estimates != null) {
                for (e in estimates) {
                    poseEstimator.addVisionMeasurement(
                        e.pose, e.timestampSeconds, e.visionMeasurementStdDevs
                    )
                }
            }
        }
        odometryLock.unlock()

        Logger.recordOutput(
            "Subsystems/Swerve/CurrentCommand",
            currentCommand?.name ?: "none"
        )

        val sampleTimestamps = modules[0].getOdometryTimestamps()
        val sampleCount = sampleTimestamps.size
        for (i in 0 until sampleCount) {
            val modulePositions = Array(4) { moduleIndex -> modules[moduleIndex].getOdometryPosition(i) }
            val moduleDeltas = Array(4) { moduleIndex ->
                SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle
                )
            }
            for (moduleIndex in 0 until 4) {
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex]
            }

            if (gyroInputs.connected) {
                rawGyroRotation = Radians.of(gyroInputs.odometryYawPositions[i])
            } else {
                val twist: Twist2d = kinematics.toTwist2d(*moduleDeltas)
                rawGyroRotation = rawGyroRotation.plus(Radians.of(twist.dtheta))
            }
            Logger.recordOutput("Subsystems/Swerve/Odometry/timestamp", sampleTimestamps[i])
            Logger.recordOutput("Subsystems/Swerve/Odometry/position", *modulePositions)

            poseEstimator.updateWithTime(sampleTimestamps[i], Rotation2d(rawGyroRotation.`in`(Radians)), modulePositions)
        }

        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.MODE != RobotMode.SIM)

        YAGSLWidget.measuredStatesObj = getModuleStates()
        YAGSLWidget.measuredChassisSpeedsObj = getChassisSpeeds()
        YAGSLWidget.robotRotationObj = getRotation()
        YAGSLWidget.updateData()

        field.setRobotPose(getPose())

        SmartDashboard.putData("field", field)
    }

    fun setFOD(fod: Boolean) {
        this.FODEnabled = fod
    }

    @AutoLogOutput(key = "Subsystems/Swerve/FOD enabled")
    fun getFOD(): Boolean {
        return FODEnabled
    }

    private fun getSpeedsFromController(): ChassisSpeeds {
        var speed = ChassisSpeeds()
        if (DriverStation.getAlliance().isPresent) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                speed = ChassisSpeeds(
                    -driverSticks.ly.asDouble * getMaxLinearSpeedMetersPerSec(),
                    -driverSticks.lx.asDouble * getMaxLinearSpeedMetersPerSec(),
                    driverSticks.rx.asDouble * getMaxAngularSpeedRadPerSec()
                )
            } else {
                speed = ChassisSpeeds(
                    driverSticks.ly.asDouble * getMaxLinearSpeedMetersPerSec(),
                    driverSticks.lx.asDouble * getMaxLinearSpeedMetersPerSec(),
                    driverSticks.rx.asDouble * getMaxAngularSpeedRadPerSec()
                )
            }
        }
        val skew = speed.omegaRadiansPerSecond * ANGULAR_VELOCITY_COEFFICIENT
        return ChassisSpeeds.fromFieldRelativeSpeeds(speed, getRotation().plus(Rotation2d(skew)))
    }

    private fun getTranslationalSpeedsFromController(angularVelocity: Double): ChassisSpeeds {
        var speed = ChassisSpeeds()
        if (DriverStation.getAlliance().isPresent) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                speed = ChassisSpeeds(
                    -driverSticks.ly.asDouble * getMaxLinearSpeedMetersPerSec(),
                    -driverSticks.lx.asDouble * getMaxLinearSpeedMetersPerSec(),
                    angularVelocity
                )
            } else {
                speed = ChassisSpeeds(
                    driverSticks.ly.asDouble * getMaxLinearSpeedMetersPerSec(),
                    driverSticks.lx.asDouble * getMaxLinearSpeedMetersPerSec(),
                    angularVelocity
                )
            }
        }
        val skew = speed.omegaRadiansPerSecond * ANGULAR_VELOCITY_COEFFICIENT
        return ChassisSpeeds.fromFieldRelativeSpeeds(speed, getRotation().plus(Rotation2d(skew)))
    }

    fun rotationLock(headingRad: DoubleSupplier): Command {
        return InstantCommand({ angleController.reset() }).andThen(
            Commands.run({
                val speeds = getTranslationalSpeedsFromController(
                    MathUtil.clamp(
                        angleController.calculate(getRotation().radians, headingRad.asDouble),
                        -ANGLE_MAX_VELOCITY.`in`(RadiansPerSecond),
                        ANGLE_MAX_VELOCITY.`in`(RadiansPerSecond)
                    )
                )
                Logger.recordOutput("Subsystems/Swerve/Rotation lock/Target angle", headingRad.asDouble)
                Logger.recordOutput("Subsystems/Swerve/Rotation lock/Angle PID out", speeds.omegaRadiansPerSecond)
                runVelocity(speeds)
            }, this)
        ).finallyDo(Runnable {
            Logger.recordOutput("Subsystems/Swerve/Rotation lock/Target angle", Double.NaN)
            Logger.recordOutput("Subsystems/Swerve/Rotation lock/Angle PID out", Double.NaN)
        }).withName("Rotation lock")
    }

    fun rotationLock(headingRad: DoubleSupplier, vx: DoubleSupplier, vy: DoubleSupplier): Command {
        return InstantCommand({ angleController.reset() }).andThen(
            Commands.run({
                var speeds = ChassisSpeeds(
                    MathUtil.clamp(vx.asDouble, -MAX_SPEED_PP.`in`(MetersPerSecond), MAX_SPEED_PP.`in`(MetersPerSecond)),
                    MathUtil.clamp(vy.asDouble, -MAX_SPEED_PP.`in`(MetersPerSecond), MAX_SPEED_PP.`in`(MetersPerSecond)),
                    MathUtil.clamp(
                        angleController.calculate(getRotation().radians, headingRad.asDouble),
                        -ANGLE_MAX_VELOCITY.`in`(RadiansPerSecond),
                        ANGLE_MAX_VELOCITY.`in`(RadiansPerSecond)
                    )
                )
                val skew = speeds.omegaRadiansPerSecond * ANGULAR_VELOCITY_COEFFICIENT
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation().plus(Rotation2d(skew)))
                Logger.recordOutput("Subsystems/Swerve/Rotation lock/Target angle", headingRad.asDouble)
                Logger.recordOutput("Subsystems/Swerve/Rotation lock/Angle PID out", speeds.omegaRadiansPerSecond)
                runVelocity(speeds)
            }, this)
        ).finallyDo(Runnable {
            Logger.recordOutput("Subsystems/Swerve/Rotation lock/Target angle", Double.NaN)
            Logger.recordOutput("Subsystems/Swerve/Rotation lock/Angle PID out", Double.NaN)
        }).withName("Rotation lock")
    }

    fun teleopDrive(): Command {
        return Commands.run({
            if (FODEnabled) {
                runVelocity(getSpeedsFromController())
            } else {
                runVelocity(
                    ChassisSpeeds(
                        driverSticks.ly.asDouble * getMaxLinearSpeedMetersPerSec(),
                        driverSticks.lx.asDouble * getMaxLinearSpeedMetersPerSec(),
                        driverSticks.rx.asDouble * getMaxAngularSpeedRadPerSec()
                    )
                )
            }
        }, this).withName("Teleop drive")
    }

    fun TrenchAlignDrive(): Command {
        return InstantCommand({
            angleController.reset()
            vyController.reset()
        }).andThen(
            Commands.run({
                val headingRad = Math.round(getRotation().radians / (Math.PI / 2.0)) * (Math.PI / 2.0)
                var trench = Inches.of(49.86).div(2.0).`in`(Meters)
                if (getPose().measureY.gte(Inches.of(316.64).div(2.0))) {
                    trench = Inches.of(316.64).minus(Inches.of(49.86).div(2.0)).`in`(Meters)
                }
                val vy = MathUtil.clamp(
                    vyController.calculate(getPose().y, trench),
                    -MAX_SPEED_PP.`in`(MetersPerSecond), MAX_SPEED_PP.`in`(MetersPerSecond)
                )
                val theta = MathUtil.clamp(
                    angleController.calculate(getRotation().radians, headingRad),
                    -ANGLE_MAX_VELOCITY.`in`(RadiansPerSecond), ANGLE_MAX_VELOCITY.`in`(RadiansPerSecond)
                )
                val speeds = ChassisSpeeds(
                    driverSticks.ly.asDouble * getMaxLinearSpeedMetersPerSec() * if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) -1 else 1,
                    vy,
                    theta
                )
                runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation()))
            }, this)
        ).withName("Trench align tele drive")
    }

    fun getAutoAlign(p: Supplier<Pose2d>): Command {
        return driveToPoseAuto(p)
            .withTimeout(2.0)
            .finallyDo(Runnable { })
    }

    fun followPath(path: PathPlannerPath): Command {
        return AutoBuilder.followPath(path).withName("Follow path: " + path.name)
    }

    fun runVelocity(speeds: ChassisSpeeds) {
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.EVENT_LOOP_TIME)
        val setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, getMaxLinearSpeedMetersPerSec())

        YAGSLWidget.desiredChassisSpeedsObj = discreteSpeeds
        Logger.recordOutput("Subsystems/Swerve/Drive/SwerveStates/Setpoints", *setpointStates)
        Logger.recordOutput("Subsystems/Swerve/Drive/SwerveChassisSpeeds/Setpoints", discreteSpeeds)
        Logger.recordOutput("Subsystems/Swerve/Drive/SwerveChassisSpeeds/SetpointAngularVel", discreteSpeeds.omegaRadiansPerSecond)

        for (i in 0 until 4) {
            modules[i].runSetpoint(setpointStates[i])
        }

        YAGSLWidget.desiredStatesObj = setpointStates
        Logger.recordOutput("Subsystems/Swerve/Drive/SwerveStates/SetpointsOptimized", *setpointStates)
    }

    fun stop() {
        for (i in 0 until 4) {
            modules[i].stop()
        }
    }

    fun stopWithX() {
        val headings = Array(4) { i -> MODULE_POSITIONS[i].angle }
        kinematics.resetHeadings(*headings)
        stop()
        runVelocity(ChassisSpeeds())
    }

    fun resetGyro(headingRad: Double) {
        poseEstimator.resetPose(Pose2d(getPose().x, getPose().y, Rotation2d(headingRad)))
    }

    private fun driveToPoseAuto(p: Supplier<Pose2d>): Command {
        return InstantCommand({
            val end = Pose2d(p.get().translation, p.get().rotation.rotateBy(Rotation2d.kCCW_90deg))
            val start = Pose2d(getPose().translation, getPathVelocityHeading(getFieldChassisSpeeds(), p.get()))

            val points: List<Waypoint> = PathPlannerPath.waypointsFromPoses(start, end)

            val constraints = PathConstraints(MAX_SPEED_PP, MAX_ACCEL_PP, MAX_ANGLE_SPEED_PP, MAX_ANGLE_ACCEL_PP)
            val path = PathPlannerPath(
                points, constraints,
                IdealStartingState(getSpeed(), getPose().rotation),
                GoalEndState(0.0, p.get().rotation)
            )
            path.preventFlipping = true

            follow = AutoBuilder.followPath(path)
        }).andThen(
            PlayCommand { follow }.withName("Follow autogenerated path"),
            FineTuneAlign(p, this).withName("Fine tune alignment")
        ).alongWith(InstantCommand({ }))
    }

    private fun getPathVelocityHeading(cs: ChassisSpeeds, target: Pose2d): Rotation2d {
        if (getSpeed() < 0.25) {
            Logger.recordOutput("Subsystems/Swerve/Align/approach", "straight line")
            val diff = target.translation.minus(getPose().translation)
            Logger.recordOutput("Subsystems/Swerve/Align/Calc/x", diff.x)
            Logger.recordOutput("Subsystems/Swerve/Align/Calc/y", diff.y)
            Logger.recordOutput("Subsystems/Swerve/Align/Calc/dir", diff.angle)

            return if (diff.norm < 0.01) target.rotation else diff.angle
        }

        Logger.recordOutput("Subsystems/Swerve/Align/approach", "velocity comp")

        val rotation = Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond)

        Logger.recordOutput("Subsystems/Swerve/Align/Calc/x", cs.vxMetersPerSecond)
        Logger.recordOutput("Subsystems/Swerve/Align/Calc/y", cs.vyMetersPerSecond)
        Logger.recordOutput("Subsystems/Swerve/Align/Calc/dir", rotation)

        return rotation
    }

    fun getAngulerVelocity(): AngularVelocity {
        return gyroInputs.yawVelocity
    }

    @AutoLogOutput(key = "Subsystems/Swerve/Speed")
    fun getSpeed(): Double {
        return Translation2d(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond).norm
    }

    fun getVelocityDir(): Rotation2d {
        return Rotation2d(Math.atan2(getChassisSpeeds().vyMetersPerSecond, getChassisSpeeds().vxMetersPerSecond))
    }

    fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command {
        return run { runCharacterization(Volts.of(0.0)) }
            .withTimeout(1.0)
            .andThen(driveSysId.quasistatic(direction))
    }

    fun sysIdDynamic(direction: SysIdRoutine.Direction): Command {
        return run { runCharacterization(Volts.of(0.0)) }.withTimeout(1.0).andThen(driveSysId.dynamic(direction))
    }

    fun steerSysIdQuasistatic(direction: SysIdRoutine.Direction): Command {
        return run { runSteerCharacterization(Volts.of(0.0)) }
            .withTimeout(1.0)
            .andThen(steerSysId.quasistatic(direction))
    }

    fun steerSysIdDynamic(direction: SysIdRoutine.Direction): Command {
        return run { runSteerCharacterization(Volts.of(0.0)) }.withTimeout(1.0).andThen(steerSysId.dynamic(direction))
    }

    fun angleSysIdQuasistatic(direction: SysIdRoutine.Direction): Command {
        return run { runAngleCharacterization(0.0) }
            .withTimeout(1.0)
            .andThen(angleSysId.quasistatic(direction))
    }

    fun angleSysIdDynamic(direction: SysIdRoutine.Direction): Command {
        return run { runAngleCharacterization(0.0) }.withTimeout(1.0).andThen(angleSysId.dynamic(direction))
    }

    @AutoLogOutput(key = "Subsystems/Swerve/Drive/SwerveStates/Measured")
    fun getModuleStates(): Array<SwerveModuleState> {
        return Array(4) { i -> modules[i].getState() }
    }

    private fun getModulePositions(): Array<SwerveModulePosition> {
        return Array(4) { i -> modules[i].getPosition() }
    }

    @AutoLogOutput(key = "Subsystems/Swerve/ChassisSpeeds/Measured")
    fun getChassisSpeeds(): ChassisSpeeds {
        return kinematics.toChassisSpeeds(*getModuleStates())
    }

    @AutoLogOutput(key = "Subsystems/Swerve/ChassisSpeeds/Field Relative")
    fun getFieldChassisSpeeds(): ChassisSpeeds {
        return ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(*getModuleStates()), getRotation())
    }

    @AutoLogOutput(key = "Subsystems/Swerve/Odometry/Robot Pose")
    fun getPose(): Pose2d {
        return poseEstimator.estimatedPosition
    }

    fun getRotation(): Rotation2d {
        return getPose().rotation
    }

    // Kotlin-friendly properties for interop (Turret, SOTMSolver, Led use drive.pose/rotation)
    @get:JvmName("getPoseKtProp")
    val pose: Pose2d
        get() = getPose()

    @get:JvmName("getRotationKtProp")
    val rotation: Rotation2d
        get() = getRotation()

    @get:JvmName("getAngulerVelocityKtProp")
    val angulerVelocity: AngularVelocity
        get() = getAngulerVelocity()

    fun setPose(pose: Pose2d) {
        poseEstimator.resetPosition(Rotation2d(rawGyroRotation.`in`(Radians)), getModulePositions(), pose)
    }

    fun getMaxLinearSpeedMetersPerSec(): Double {
        return MAX_SPEED_PP.`in`(MetersPerSecond)
    }

    fun getMaxAngularSpeedRadPerSec(): Double {
        return MAX_ANGLE_SPEED_PP.`in`(RadiansPerSecond)
    }

    fun runCharacterization(output: Voltage) {
        for (i in 0 until 4) {
            modules[i].runCharacterization(output)
        }
    }

    fun runSteerCharacterization(output: Voltage) {
        for (i in 0 until 4) {
            modules[i].runSteerCharacterization(output)
        }
    }

    fun runAngleCharacterization(output: Double) {
        val discreteSpeeds = ChassisSpeeds.discretize(ChassisSpeeds(0.0, 0.0, 1.0), Constants.EVENT_LOOP_TIME)
        val setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds)

        for (i in 0 until 4) {
            modules[i].runCharacterization(Volts.of(output), Radians.of(setpointStates[i].angle.radians))
        }
    }

    companion object {
        @JvmField
        val odometryLock: Lock = ReentrantLock()
    }
}
