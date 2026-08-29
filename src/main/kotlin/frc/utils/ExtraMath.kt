package frc.utils

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile.State
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.util.Color
import frc.robot.constants.TurretConstants
import org.littletonrobotics.junction.Logger
import java.util.ArrayList
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sqrt

object ExtraMath {

    @JvmStatic
    fun calculateTurretAngleFromCANCoderDegrees(e1: Double, e2: Double): Double {
        var difference = e2 - e1
        if (difference > 180) {
            difference -= 360
        }
        if (difference < -180) {
            difference += 360
        }
        difference *= TurretConstants.SLOPE

        val e1Rotations = (difference * TurretConstants.TURRET_MAIN_GEAR_TEETH / TurretConstants.TURRET_ENCODER_1_GEAR_TEETH) / 360.0
        val e1RotationsFloored = floor(e1Rotations)
        var turretAngle = (e1RotationsFloored * 360.0 + e1) * (TurretConstants.TURRET_ENCODER_1_GEAR_TEETH / TurretConstants.TURRET_MAIN_GEAR_TEETH)

        Logger.recordOutput("fahhh/angle", turretAngle)
        if (turretAngle - difference < -40) {
            turretAngle += TurretConstants.TURRET_ENCODER_1_GEAR_TEETH / TurretConstants.TURRET_MAIN_GEAR_TEETH * 360.0
        } else if (turretAngle - difference > 40) {
            turretAngle -= TurretConstants.TURRET_ENCODER_1_GEAR_TEETH / TurretConstants.TURRET_MAIN_GEAR_TEETH * 360.0
        }
        Logger.recordOutput("fahhh/dif", difference)
        return turretAngle
    }

    @JvmStatic
    fun <T : Measure<out Unit>> clamp(`val`: T, min: T, max: T): T {
        if (`val`.baseUnitMagnitude() >= max.baseUnitMagnitude()) {
            return max
        }
        if (`val`.baseUnitMagnitude() <= min.baseUnitMagnitude()) {
            return min
        }
        return `val`
    }

    @JvmStatic
    fun getAngleToPos(target: Translation2d, curr: Translation2d): Angle {
        return Radians.of(atan2(target.y - curr.y, target.x - curr.x))
    }

    @JvmStatic
    fun mean(vararg `in`: Double): Double {
        var sum = 0.0
        for (i in `in`) {
            sum += i
        }
        return sum / `in`.size
    }

    /**
     * @param in values to compare
     * @return value with smallest absolute value
     */
    @JvmStatic
    fun lesser(vararg `in`: Double): Double {
        var lesser = Double.POSITIVE_INFINITY
        var lesserAbs = Double.POSITIVE_INFINITY
        for (v in `in`) {
            if (abs(v) < lesserAbs) {
                lesser = v
                lesserAbs = abs(v)
            }
        }
        return lesser
    }

    /**
     * @param in values to compare
     * @return value with largest absolute value
     */
    @JvmStatic
    fun greater(vararg `in`: Double): Double {
        var greater = Double.NEGATIVE_INFINITY
        var greaterAbs = 0.0
        for (v in `in`) {
            if (abs(v) > greaterAbs) {
                greater = v
                greaterAbs = abs(v)
            }
        }
        return greater
    }

    @JvmStatic
    fun min(vararg `in`: Double): Double {
        var lesser = Double.POSITIVE_INFINITY
        var lesserAbs = Double.POSITIVE_INFINITY
        for (v in `in`) {
            if (v < lesserAbs) {
                lesser = v
                lesserAbs = v
            }
        }
        return lesser
    }

    @JvmStatic
    fun max(vararg `in`: Double): Double {
        var greater = Double.NEGATIVE_INFINITY
        var greaterAbs = 0.0
        for (v in `in`) {
            if (v > greaterAbs) {
                greater = v
                greaterAbs = v
            }
        }
        return greater
    }

    /**
     * wraps input to be within the range [0, modulus)
     */
    @JvmStatic
    fun wrap(input: Double, modulus: Double): Double {
        return ((input % modulus) + modulus) % modulus
    }

    @JvmStatic
    fun isNearState(expected: State, actual: State, tolerance: State): Boolean {
        return MathUtil.isNear(expected.position, actual.position, tolerance.position) &&
            MathUtil.isNear(expected.velocity, actual.velocity, tolerance.velocity)
    }

    /**
     * round to n decimal places
     */
    @JvmStatic
    fun roundToPoint(`val`: Double, point: Int): Double {
        return (`val` * 10.0.pow(point.toDouble())).toInt() / 10.0.pow(point.toDouble())
    }

    /**
     * get rotation2D angle from 0,0 to u,v
     */
    @JvmStatic
    fun getAngle(u: Double, v: Double): Rotation2d {
        return Rotation2d(atan2(v, u))
    }

    /**
     * get distance between 0,0 and x,y
     */
    @JvmStatic
    fun getMagnitude(x: Double, y: Double): Double {
        return sqrt(x.pow(2) + y.pow(2))
    }

    /**
     * get tilt of robot
     * @param angle 3d gyro angle of bot
     * @return double[2] index 0 is direction(-pi to pi), index 1 is tilt angle
     */
    @JvmStatic
    fun getTip(angle: Rotation3d): DoubleArray {
        val out = DoubleArray(2)
        out[0] = atan2(-angle.x, -angle.y)
        out[1] = hypot(angle.x, angle.y)
        return out
    }

    /**
     * process input value
     * curve function graphed
     * [here](https://www.desmos.com/calculator/fjuc4iqjqt)
     */
    @JvmStatic
    fun processInput(`val`: Double?, multiplier: Double?, square: Double?, deadZone: Double?): Double {
        var out = `val`!!

        if (square != null) {
            if (deadZone != null && deadZone > 0) {
                out = sign(`val`) * (((1 / (-deadZone + 1)) * abs(`val`) - (deadZone / (-deadZone + 1))).pow(square))
            } else {
                out = sign(`val`) * abs(`val`).pow(square)
            }
        }
        if (deadZone != null) {
            out = MathUtil.applyDeadband(out, deadZone)
        }
        if (multiplier != null) {
            out *= multiplier
        }

        return out
    }

    @JvmStatic
    fun remap(`val`: Double, inMin: Double, inMax: Double, outMin: Double, outMax: Double): Double {
        return ((`val` - inMin) / (inMax - inMin) * (outMax - outMin)) + outMin
    }

    @JvmStatic
    fun lerp(start: Double, end: Double, `val`: Double): Double {
        return (end - start) * `val` + start
    }

    /**
     * lerp rgb color
     */
    @JvmStatic
    fun colLerp(start: Color, end: Color, `val`: Double): Color {
        val r = lerp(start.red, end.red, `val`)
        val g = lerp(start.green, end.green, `val`)
        val b = lerp(start.blue, end.blue, `val`)
        return Color(r, g, b)
    }

    @JvmStatic
    fun rgbToHsv(`in`: Color): Color {
        val hsv = FloatArray(3)
        java.awt.Color.RGBtoHSB((`in`.red * 255).toInt(), (`in`.green * 255).toInt(), (`in`.blue * 255).toInt(), hsv)
        return Color(hsv[0].toDouble(), hsv[1].toDouble(), hsv[2].toDouble())
    }

    /**
     * get nearest pose from array to current
     */
    @JvmStatic
    fun getNearestPose(poses: Array<Pose2d>, current: Pose2d): Pose2d {
        var min = Double.MAX_VALUE
        var out = poses[0]
        for (pose in poses) {
            var dist = current.translation.getDistance(pose.translation)
            dist += abs(current.rotation.minus(pose.rotation).radians)
            dist = abs(dist / 2.0)
            if (dist < min) {
                min = dist
                out = pose
            }
        }
        return out
    }

    /**
     * get translation distance between two pose2Ds
     */
    @JvmStatic
    fun getDistance(a: Pose2d, b: Pose2d): Double {
        return a.translation.getDistance(b.translation)
    }

    /**
     * check if two poses are within tolerance in translation and rotation
     */
    @JvmStatic
    fun poseWithinTolerance(a: Pose2d, b: Pose2d, toleranceLinear: Double, toleranceAngular: Double): Boolean {
        val dist = getDistance(a, b)
        val angleDiff = abs(a.rotation.minus(b.rotation).radians)
        return dist < toleranceLinear && angleDiff < toleranceAngular
    }

    @JvmStatic
    fun normalizeCol(`in`: Color): Color {
        val brt = `in`.red / 255.0 + `in`.green / 255.0 + `in`.blue / 255.0
        return Color(`in`.red / brt, `in`.green / brt, `in`.blue / brt)
    }

    /**
     * Derivitive class
     */
    class Derivitive {
        private var oldValue: Double = 0.0
        private var init: Boolean = false

        constructor(initMesure: Double) {
            oldValue = initMesure
            init = true
        }

        constructor() {
            init = false
        }

        fun calculate(mesurement: Double, dt: Double): Double {
            if (!init) {
                oldValue = mesurement
                init = true
            }
            val out = (mesurement - oldValue) / dt
            oldValue = mesurement
            return out
        }

        fun reset(initMesure: Double) {
            oldValue = initMesure
        }
    }

    class MovingAverageFilter(private val taps: Int) {
        private val window: ArrayList<Double> = ArrayList()

        fun calculate(`in`: Double): Double {
            window.add(`in`)
            if (window.size > taps) {
                window.removeAt(0)
            }
            var t = 0.0
            for (x in window) {
                t += x
            }
            t /= window.size
            return t
        }

        fun reset() {
            window.clear()
        }
    }
}
