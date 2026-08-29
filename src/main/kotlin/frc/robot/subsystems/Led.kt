package frc.robot.subsystems

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.units.Units.Percent
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.AddressableLEDBufferView
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.Constants
import frc.robot.constants.Constants.RobotMode
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.launcher.Launcher
import frc.robot.subsystems.swerve.Drive
import frc.robot.subsystems.turret.Turret
import frc.utils.ExtraMath
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import java.util.function.BooleanSupplier

class Led(
    private val launcher: Launcher,
    private val hood: Hood,
    private val turret: Turret,
    private val drive: Drive,
    private val manualSupplier: BooleanSupplier
) : SubsystemBase() {

    private val led = AddressableLED(1)
    private val buffer = AddressableLEDBuffer(70 + 5)
    private val sideBuffer = AddressableLEDBufferView(buffer, 70, 70 + 4)
    private val turretBuffer = AddressableLEDBufferView(buffer, 0, 70)

    private val turretBufferLength: Int = turretBuffer.length
    private val lEDTurretZeroOffset: Int = turretBufferLength - 16

    private val coolTurretTrackLedThing = LoggedNetworkBoolean("Overrides/Cool Turret Track Led Thing", true)
    private val coolTurretTrackLedThingButItAlsoHelpsWithTurretDebug = LoggedNetworkBoolean("Overrides/Cool Turret Track Led Thing But It Also Helps With Turret Debug", false)

    private val rainbow = LoggedNetworkBoolean("Overrides/LED Override", false)
    private val disable = LoggedNetworkBoolean("Overrides/LED Disable", false)

    private val readyDebouncer = Debouncer(0.2)

    init {
        led.setLength(buffer.length)
        led.start()
    }

    override fun periodic() {
        var turretState: Color = if (readyDebouncer.calculate(turret.isReady() && launcher.isReady() && hood.isReady())) Color.kGreen else Color.kBlack
        if (manualSupplier.asBoolean) {
            turretState = Color.kWhite
        }
        if (!hood.isHomed()) {
            turretState = Color.kRed
        }
        if (hood.isHoming()) {
            turretState = Color.kOrange
        }

        var turretAbsolutePosIndexLed = (ExtraMath.wrap(turret.getAbsoluteAngle().times(-1.0).`in`(Rotations), 1.0) * turretBufferLength).toInt()
        turretAbsolutePosIndexLed += lEDTurretZeroOffset
        turretAbsolutePosIndexLed %= turretBufferLength
        var turretPosIndexLed = (ExtraMath.wrap(turret.getAngle().times(-1.0).`in`(Rotations), 1.0) * turretBufferLength).toInt()
        turretPosIndexLed += lEDTurretZeroOffset
        turretPosIndexLed %= turretBufferLength
        val sideState: Color = if (drive.getFOD()) Color() else Color.kBlue

        if (rainbow.get()) {
            LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Hertz.of(0.5)).applyTo(turretBuffer)
            LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Hertz.of(0.5)).applyTo(sideBuffer)
        } else {
            LEDPattern.solid(turretState).atBrightness(Percent.of(50.0)).applyTo(turretBuffer)

            if (coolTurretTrackLedThing.get() && !coolTurretTrackLedThingButItAlsoHelpsWithTurretDebug.get()) {
                turretBuffer.setLED(turretPosIndexLed, Color.kMagenta)
                for (i in 1 until 5) {
                    var ledIndexPlus = ExtraMath.wrap((turretPosIndexLed + i).toDouble(), turretBufferLength.toDouble()).toInt()
                    var ledIndexMinus = ExtraMath.wrap((turretPosIndexLed - i).toDouble(), turretBufferLength.toDouble()).toInt()
                    if (ledIndexMinus < 0) {
                        ledIndexMinus += turretBufferLength
                    }

                    turretBuffer.setLED(ledIndexPlus, Color.kMagenta)
                    turretBuffer.setLED(ledIndexMinus, Color.kMagenta)
                }
            }

            if (coolTurretTrackLedThingButItAlsoHelpsWithTurretDebug.get()) {
                turretBuffer.setLED(turretPosIndexLed, Color.kRed)
                turretBuffer.setLED(turretAbsolutePosIndexLed, Color.kGreen)
            }

            LEDPattern.solid(sideState).atBrightness(Percent.of(50.0)).applyTo(sideBuffer)
        }

        if (disable.get()) {
            LEDPattern.solid(Color()).applyTo(buffer)
        }

        led.setData(buffer)

        // only log all leds in replay, as toHexString() is slow
        if (Constants.MODE == RobotMode.REPLAY) {
            val leds = Array(buffer.length) { i -> buffer.getLED(i).toHexString() }
            Logger.recordOutput("Leds/list", leds)
        }
        Logger.recordOutput("Leds/turret", turretState)
        Logger.recordOutput("Leds/side", sideState)
    }
}
