package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer

object YAGSLWidget {
    private val moduleCountPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/moduleCount")
        .publish()

    private val measuredStatesArrayPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleArrayTopic("swerve/measuredStates")
        .publish()

    private val desiredStatesArrayPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleArrayTopic("swerve/desiredStates")
        .publish()

    private val measuredChassisSpeedsArrayPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleArrayTopic("swerve/measuredChassisSpeeds")
        .publish()

    private val desiredChassisSpeedsArrayPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleArrayTopic("swerve/desiredChassisSpeeds")
        .publish()

    private val robotRotationPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/robotRotation")
        .publish()

    private val maxAngularVelocityPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/maxAngularVelocity")
        .publish()

    private val measuredStatesStruct = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getStructArrayTopic("swerve/advantagescope/currentStates", SwerveModuleState.struct)
        .publish()

    private val desiredStatesStruct = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getStructArrayTopic("swerve/advantagescope/desiredStates", SwerveModuleState.struct)
        .publish()

    private val measuredChassisSpeedsStruct = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getStructTopic("swerve/advantagescope/measuredChassisSpeeds", ChassisSpeeds.struct)
        .publish()

    private val desiredChassisSpeedsStruct = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getStructTopic("swerve/advantagescope/desiredChassisSpeeds", ChassisSpeeds.struct)
        .publish()

    private val robotRotationStruct = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getStructTopic("swerve/advantagescope/robotRotation", Rotation2d.struct)
        .publish()

    private val wheelLocationsArrayPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleArrayTopic("swerve/wheelLocation")
        .publish()

    private val maxSpeedPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/maxSpeed")
        .publish()

    private val rotationUnitPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getStringTopic("swerve/rotationUnit")
        .publish()

    private val sizeLeftRightPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/sizeLeftRight")
        .publish()

    private val sizeFrontBackPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/sizeFrontBack")
        .publish()

    private val forwardDirectionPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getStringTopic("swerve/forwardDirection")
        .publish()

    private val odomCycleTime = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/odomCycleMS")
        .publish()

    private val ctrlCycleTime = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/controlCycleMS")
        .publish()

    private val odomTimer = Timer()
    private val ctrlTimer = Timer()

    @JvmField var measuredStatesObj: Array<SwerveModuleState> = Array(4) { SwerveModuleState() }
    @JvmField var desiredStatesObj: Array<SwerveModuleState> = Array(4) { SwerveModuleState() }
    @JvmField var measuredChassisSpeedsObj: ChassisSpeeds = ChassisSpeeds()
    @JvmField var desiredChassisSpeedsObj: ChassisSpeeds = ChassisSpeeds()
    @JvmField var robotRotationObj: Rotation2d = Rotation2d()

    @JvmField var isSimulation: Boolean = RobotBase.isSimulation()
    @JvmField var moduleCount: Int = 0
    @JvmField var wheelLocations: DoubleArray = doubleArrayOf()
    @JvmField var measuredStates: DoubleArray = DoubleArray(8)
    @JvmField var desiredStates: DoubleArray = DoubleArray(8)
    @JvmField var robotRotation: Double = 0.0
    @JvmField var maxSpeed: Double = 0.0
    @JvmField var rotationUnit: String = "degrees"
    @JvmField var sizeLeftRight: Double = Units.inchesToMeters(25.0)
    @JvmField var sizeFrontBack: Double = Units.inchesToMeters(25.0)
    @JvmField var forwardDirection: String = "up"
    @JvmField var maxAngularVelocity: Double = 0.0
    @JvmField var measuredChassisSpeeds: DoubleArray = DoubleArray(3)
    @JvmField var desiredChassisSpeeds: DoubleArray = DoubleArray(3)
    @JvmField var updateSettings: Boolean = true

    @JvmStatic
    fun startCtrlCycle() {
        if (ctrlTimer.isRunning) {
            ctrlTimer.reset()
        } else {
            ctrlTimer.start()
        }
    }

    @JvmStatic
    fun endCtrlCycle() {
        if (DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled() || DriverStation.isTestEnabled()) {
            ctrlCycleTime.set(ctrlTimer.get() * 1000)
        }
        ctrlTimer.reset()
    }

    @JvmStatic
    fun startOdomCycle() {
        if (odomTimer.isRunning) {
            odomTimer.reset()
        } else {
            odomTimer.start()
        }
    }

    @JvmStatic
    fun endOdomCycle() {
        if (DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled() || DriverStation.isTestEnabled()) {
            odomCycleTime.set(odomTimer.get() * 1000)
        }
        odomTimer.reset()
    }

    @JvmStatic
    fun updateSwerveTelemetrySettings() {
        updateSettings = false
        wheelLocationsArrayPublisher.set(wheelLocations)
        maxSpeedPublisher.set(maxSpeed)
        rotationUnitPublisher.set(rotationUnit)
        sizeLeftRightPublisher.set(sizeLeftRight)
        sizeFrontBackPublisher.set(sizeFrontBack)
        forwardDirectionPublisher.set(forwardDirection)
    }

    @JvmStatic
    fun updateData() {
        if (updateSettings) {
            updateSwerveTelemetrySettings()
        }

        measuredChassisSpeeds[0] = measuredChassisSpeedsObj.vxMetersPerSecond
        measuredChassisSpeeds[1] = measuredChassisSpeedsObj.vxMetersPerSecond
        measuredChassisSpeeds[2] = Math.toDegrees(measuredChassisSpeedsObj.omegaRadiansPerSecond)

        desiredChassisSpeeds[0] = desiredChassisSpeedsObj.vxMetersPerSecond
        desiredChassisSpeeds[1] = desiredChassisSpeedsObj.vyMetersPerSecond
        desiredChassisSpeeds[2] = Math.toDegrees(desiredChassisSpeedsObj.omegaRadiansPerSecond)

        robotRotation = robotRotationObj.degrees

        for (i in measuredStatesObj.indices) {
            val state = measuredStatesObj[i]
            measuredStates[i * 2] = state.angle.degrees
            measuredStates[i * 2 + 1] = state.speedMetersPerSecond
        }

        for (i in desiredStatesObj.indices) {
            val state = desiredStatesObj[i]
            desiredStates[i * 2] = state.angle.degrees
            desiredStates[i * 2 + 1] = state.speedMetersPerSecond
        }

        moduleCountPublisher.set(moduleCount.toDouble())
        measuredStatesArrayPublisher.set(measuredStates)
        desiredStatesArrayPublisher.set(desiredStates)
        robotRotationPublisher.set(robotRotation)
        maxAngularVelocityPublisher.set(maxAngularVelocity)

        measuredChassisSpeedsArrayPublisher.set(measuredChassisSpeeds)
        desiredChassisSpeedsArrayPublisher.set(desiredChassisSpeeds)

        desiredStatesStruct.set(desiredStatesObj)
        measuredStatesStruct.set(measuredStatesObj)
        desiredChassisSpeedsStruct.set(desiredChassisSpeedsObj)
        measuredChassisSpeedsStruct.set(measuredChassisSpeedsObj)
        robotRotationStruct.set(robotRotationObj)
    }
}
