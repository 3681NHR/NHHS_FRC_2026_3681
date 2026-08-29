package frc.robot.subsystems

import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import frc.robot.constants.Constants
import frc.robot.constants.Constants.RobotMode
import frc.utils.ExtraMath
import org.littletonrobotics.junction.Logger

object LaunchLUT {
    /**
     * hub LUT values
     * must be sorted from smallest dist to largest
     */
    @JvmField
    val LUTHub: Array<ShotParams> = if (Constants.MODE == RobotMode.SIM) arrayOf(
        ShotParams(Meters.of(1.516), Degrees.of(25.0), RPM.of(1150.0), Seconds.of(0.8)),
        ShotParams(Meters.of(1.989), Degrees.of(27.0), RPM.of(1228.0), Seconds.of(0.9)),
        ShotParams(Meters.of(2.3), Degrees.of(30.0), RPM.of(1228.0), Seconds.of(0.91)),
        ShotParams(Meters.of(2.96), Degrees.of(30.0), RPM.of(1300.0), Seconds.of(0.92)),
        ShotParams(Meters.of(3.77), Degrees.of(32.0), RPM.of(1450.0), Seconds.of(1.1)),
        ShotParams(Meters.of(3.69), Degrees.of(32.0), RPM.of(1400.0), Seconds.of(1.11)),
        ShotParams(Meters.of(5.33), Degrees.of(35.0), RPM.of(1550.0), Seconds.of(1.2)),
    ) else arrayOf(
        ShotParams(Meters.of(1.6), Degrees.of(28.0), RPM.of(2500.0), Seconds.of(0.8)),
        ShotParams(Meters.of(2.42), Degrees.of(32.0), RPM.of(2750.0), Seconds.of(1.05)),
        ShotParams(Meters.of(2.9), Degrees.of(35.0), RPM.of(2750.0), Seconds.of(1.05)),
        ShotParams(Meters.of(3.425), Degrees.of(35.0), RPM.of(3000.0), Seconds.of(1.1)),
        ShotParams(Meters.of(4.07), Degrees.of(35.0), RPM.of(3250.0), Seconds.of(0.95)),
        ShotParams(Meters.of(4.55), Degrees.of(37.0), RPM.of(3300.0), Seconds.of(0.95)),
        ShotParams(Meters.of(5.04), Degrees.of(40.0), RPM.of(3500.0), Seconds.of(1.32)),
    )

    /**
     * pass LUT values
     * must be sorted from smallest dist to largest
     */
    @JvmField
    val LUTPass: Array<ShotParams> = if (Constants.MODE == RobotMode.SIM) arrayOf(
        ShotParams(Meters.of(1.516), Degrees.of(25.0), RPM.of(1150.0), Seconds.of(0.8)),
        ShotParams(Meters.of(1.989), Degrees.of(27.0), RPM.of(1228.0), Seconds.of(0.9)),
        ShotParams(Meters.of(2.3), Degrees.of(30.0), RPM.of(1228.0), Seconds.of(0.91)),
        ShotParams(Meters.of(2.96), Degrees.of(30.0), RPM.of(1300.0), Seconds.of(0.92)),
        ShotParams(Meters.of(3.77), Degrees.of(32.0), RPM.of(1450.0), Seconds.of(1.1)),
        ShotParams(Meters.of(3.69), Degrees.of(32.0), RPM.of(1400.0), Seconds.of(1.11)),
        ShotParams(Meters.of(5.33), Degrees.of(35.0), RPM.of(1550.0), Seconds.of(1.2)),
        ShotParams(Meters.of(7.41), Degrees.of(45.0), RPM.of(1700.0), Seconds.of(1.4)),
        ShotParams(Meters.of(9.86), Degrees.of(45.0), RPM.of(1900.0), Seconds.of(1.9)),
    ) else arrayOf(
        ShotParams(Meters.of(3.3), Degrees.of(30.0), RPM.of(2250.0), Seconds.of(0.9)),
        ShotParams(Meters.of(4.48), Degrees.of(40.0), RPM.of(2500.0), Seconds.of(0.95)),
        ShotParams(Meters.of(5.52), Degrees.of(45.0), RPM.of(3000.0), Seconds.of(1.05)),
        ShotParams(Meters.of(6.45), Degrees.of(45.0), RPM.of(3500.0), Seconds.of(1.25)),
        ShotParams(Meters.of(7.53), Degrees.of(45.0), RPM.of(4500.0), Seconds.of(1.5)),
        ShotParams(Meters.of(10.5), Degrees.of(45.0), RPM.of(6000.0), Seconds.of(1.8)),
        ShotParams(Meters.of(10.6), Degrees.of(45.0), RPM.of(6000.0), Seconds.of(1.8)),
    )

    /**
     * get data from LUT at given distance val
     * @param dist - value to lookup
     * @param lerp - weather to lerp between closest values or return nearest entry(will always return above dist)
     * @param LUT - array of params to use, sorted by distance
     * @return - array with data, [hood angle, launcher speed, TOF]
     */
    @JvmStatic
    fun get(dist: Distance, lerp: Boolean, LUT: Array<ShotParams>): ShotParams {
        var out = ShotParams(Meters.of(0.0), Degrees.of(0.0), RPM.of(0.0), Seconds.of(0.0))
        if (LUT.size > 1 || !dist.baseUnitMagnitude().isFinite()) {
            var i = 0
            while (LUT[i].dist().lt(dist)) {
                i++
                if (i >= LUT.size) {
                    Logger.recordOutput("lut/extrapolate", true)
                    if (!lerp && LUT.size >= 2) {
                        return LUT[LUT.size - 1]
                    } else {
                        val factor = dist.minus(LUT[LUT.size - 2].dist())
                            .div(LUT[LUT.size - 1].dist().minus(LUT[LUT.size - 2].dist()))
                            .magnitude()
                        return ShotParams(
                            dist,
                            Radians.of(ExtraMath.lerp(LUT[LUT.size - 2].hoodAngle().`in`(Radians), LUT[LUT.size - 1].hoodAngle().`in`(Radians), factor)),
                            RPM.of(ExtraMath.lerp(LUT[LUT.size - 2].speed().`in`(RPM), LUT[LUT.size - 1].speed().`in`(RPM), factor)),
                            Seconds.of(ExtraMath.lerp(LUT[LUT.size - 2].time().`in`(Seconds), LUT[LUT.size - 1].time().`in`(Seconds), factor))
                        )
                    }
                }
            }
            Logger.recordOutput("lut/extrapolate", false)

            if (lerp && i > 0) {
                val factor = dist.minus(LUT[i - 1].dist())
                    .div(LUT[i].dist().minus(LUT[i - 1].dist()))
                    .magnitude()

                out = ShotParams(
                    dist,
                    Radians.of(ExtraMath.lerp(LUT[i - 1].hoodAngle().`in`(Radians), LUT[i].hoodAngle().`in`(Radians), factor)),
                    RPM.of(ExtraMath.lerp(LUT[i - 1].speed().`in`(RPM), LUT[i].speed().`in`(RPM), factor)),
                    Seconds.of(ExtraMath.lerp(LUT[i - 1].time().`in`(Seconds), LUT[i].time().`in`(Seconds), factor))
                )
            } else {
                out = LUT[i]
            }

            Logger.recordOutput("lut/index", i)
        }
        Logger.recordOutput("lut/dist", dist)
        Logger.recordOutput("lut/params/dist", out.dist())
        Logger.recordOutput("lut/params/hood", out.hoodAngle())
        Logger.recordOutput("lut/params/speed", out.speed())
        Logger.recordOutput("lut/params/tof", out.time())

        return out
    }

    /**
     * get slope of data from LUT at given distance val
     */
    @JvmStatic
    fun getSlope(dist: Distance, LUT: Array<ShotParams>): ShotParams {
        var out = ShotParams(Meters.of(1.0), Degrees.of(1.0), RPM.of(1.0), Seconds.of(1.0))

        if (LUT.size > 1) {
            var i = 0
            while (LUT[i].dist().lt(dist)) {
                i++
                if (i >= LUT.size) {
                    Logger.recordOutput("lut/slope/extrapolate", true)

                    val run = LUT[LUT.size - 2].dist().`in`(Meters) - LUT[LUT.size - 1].dist().`in`(Meters)
                    return ShotParams(
                        Meters.of(1.0),
                        Radians.of((LUT[LUT.size - 2].hoodAngle().`in`(Radians) - LUT[LUT.size - 1].hoodAngle().`in`(Radians)) / run),
                        RPM.of((LUT[LUT.size - 2].speed().`in`(RPM) - LUT[LUT.size - 1].speed().`in`(RPM)) / run),
                        Seconds.of((LUT[LUT.size - 2].time().`in`(Seconds) - LUT[LUT.size - 1].time().`in`(Seconds)) / run)
                    )
                }
            }
            Logger.recordOutput("lut/slope/extrapolate", false)

            i = Math.max(i, 1)

            val run = LUT[i - 1].dist().`in`(Meters) - LUT[i].dist().`in`(Meters)
            out = ShotParams(
                Meters.of(1.0),
                Radians.of((LUT[i - 1].hoodAngle().`in`(Radians) - LUT[i].hoodAngle().`in`(Radians)) / run),
                RPM.of((LUT[i - 1].speed().`in`(RPM) - LUT[i].speed().`in`(RPM)) / run),
                Seconds.of((LUT[i - 1].time().`in`(Seconds) - LUT[i].time().`in`(Seconds)) / run)
            )

            Logger.recordOutput("lut/slope/index", i)
        }
        Logger.recordOutput("lut/slope/dist", dist)

        Logger.recordOutput("lut/slope/params/dist", out.dist())
        Logger.recordOutput("lut/slope/params/hood", out.hoodAngle())
        Logger.recordOutput("lut/slope/params/speed", out.speed())
        Logger.recordOutput("lut/slope/params/tof", out.time())

        return out
    }

    data class ShotParams(
        private val distValue: Distance,
        private val hoodAngleValue: Angle,
        private val speedValue: AngularVelocity,
        private val timeValue: Time
    ) {
        fun dist(): Distance = distValue
        fun hoodAngle(): Angle = hoodAngleValue
        fun speed(): AngularVelocity = speedValue
        fun time(): Time = timeValue

        // Kotlin property accessors for idiomatic use
        val dist: Distance get() = distValue
        val hoodAngle: Angle get() = hoodAngleValue
        val speed: AngularVelocity get() = speedValue
        val time: Time get() = timeValue
    }
}
