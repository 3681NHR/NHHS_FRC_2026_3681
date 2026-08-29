package frc.robot.subsystems

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.TurretConstants
import frc.robot.subsystems.LaunchLUT.ShotParams
import frc.robot.subsystems.swerve.Drive
import frc.utils.ExtraMath

class SOTMSolver private constructor() : SubsystemBase() {

    private var params: ShotParams = ShotParams(Meters.of(0.0), Radians.of(0.0), RPM.of(0.0), Seconds.of(0.0))
    @JvmField
    var target: Translation2d = Translation2d()
    private var drive: Drive? = null
    private var turretAngle: Angle = Radians.of(0.0)

    private var LUT: Array<ShotParams> = LaunchLUT.LUTHub

    override fun periodic() {
        calculate()
    }

    fun setTarget(targ: Translation2d) {
        this.target = targ
    }

    fun getTarget(): Translation2d = target

    fun setDrive(drive: Drive) {
        this.drive = drive
    }

    fun calculate() {
        val d = drive!! // preserve NPE if not initialized, matching Java behavior
        val curr = d.pose.translation.plus(TurretConstants.TURRET_OFFSET.toTranslation2d().rotateBy(d.pose.rotation))

        var dist = curr.getDistance(target)
        var newDist = 0.0
        var vec = Translation2d()
        params = LaunchLUT.get(Meters.of(dist), true, LUT)

        for (i in 0 until 5) {
            if (Math.abs(dist - newDist) <= Units.inchesToMeters(10.0)) break
            dist = newDist

            vec = curr.plus(
                Translation2d(
                    d.getFieldChassisSpeeds().vxMetersPerSecond * params.time().`in`(Seconds),
                    d.getFieldChassisSpeeds().vyMetersPerSecond * params.time().`in`(Seconds)
                )
            )

            turretAngle = ExtraMath.getAngleToPos(target, vec)
            newDist = vec.getDistance(target)
            params = LaunchLUT.get(Meters.of(newDist), true, LUT)
        }
    }

    /**
     * calculate
     *
     * requires the following:
     *  - slope of time is never 0
     *  - time is never 0
     */
    fun calculate2() {
        val d = drive!!
        val curr = d.pose.translation

        params = LaunchLUT.get(Meters.of(curr.getDistance(target)), true, LUT)
        if (params.time().`in`(Seconds) <= 0) {
            System.err.println("SOTM calculation canceled(time of flight was 0)")
            return
        }

        val vel = Translation2d(
            d.getFieldChassisSpeeds().vxMetersPerSecond,
            d.getFieldChassisSpeeds().vyMetersPerSecond
        )

        val shotVel = Translation2d(
            Math.cos(ExtraMath.getAngleToPos(target, curr).`in`(Radians)),
            Math.sin(ExtraMath.getAngleToPos(target, curr).`in`(Radians))
        ).times(curr.getDistance(target) / params.time().`in`(Seconds))

        val targetVel = shotVel.minus(vel)
        turretAngle = if (targetVel.getDistance(Translation2d()) == 0.0) {
            // point at target as fallback
            ExtraMath.getAngleToPos(target, curr)
        } else {
            Radians.of(targetVel.angle.radians)
        }

        val targetV = targetVel.getDistance(Translation2d())
        var currentV = params.dist().`in`(Meters) / params.time().`in`(Seconds)
        var dist = params.dist().`in`(Meters)

        for (i in 0 until 10) {
            if (Math.abs(currentV - targetV) <= 0.005) break
            // d/dx(dist/time)
            // = dist'*time - dist*time' / time^2
            // = -dist*time' / time^2
            // -dist feels wrong
            val dv = (-dist * LaunchLUT.getSlope(Meters.of(dist), LUT).time().`in`(Seconds)) / Math.pow(params.time().`in`(Seconds), 2.0)

            // newtons method the goat
            dist -= (currentV - targetV) / dv

            params = LaunchLUT.get(Meters.of(dist), true, LUT)

            if (params.time().`in`(Seconds) <= 0) {
                System.err.println("SOTM calculation canceled(time of flight was 0)")
                return
            }
            currentV = dist / params.time().`in`(Seconds)
        }
    }

    @JvmOverloads
    fun getParams(refresh: Boolean): ShotParams {
        if (refresh) {
            calculate()
        }
        return params
    }

    fun getAngle(refresh: Boolean): Angle {
        if (refresh) {
            calculate()
        }
        return turretAngle
    }

    fun setLUT(newLUT: Array<ShotParams>) {
        LUT = newLUT
    }

    companion object {
        @Volatile
        private var instance: SOTMSolver? = null

        @Synchronized
        @JvmStatic
        fun getInstance(): SOTMSolver {
            if (instance == null) {
                instance = SOTMSolver()
            }
            return instance!!
        }
    }
}
