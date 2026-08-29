package frc.utils.motorWrappers

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.hardware.TalonFX as CtreTalonFX
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import frc.robot.constants.Constants
import org.littletonrobotics.junction.Logger
import java.util.HashMap
import java.util.HashSet
import java.util.LinkedHashSet
import java.util.Locale

class TalonFX : CtreTalonFX {

    private val disconnectKey: String
    private val tempKey: String

    /**
     * Constructs a new Talon FX motor controller object.
     *
     * Constructs the device using the default CAN bus for the system
     * (see [CANBus.CANBus]).
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner
     */
    constructor(deviceId: Int) : this(deviceId, CANBus())

    /**
     * Constructs a new Talon FX motor controller object.
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner
     * @param canbus   Name of the CAN bus this device is on.
     * @deprecated Constructing devices with a CAN bus string is deprecated for
     *             removal in the 2027 season. Construct devices using a [CANBus]
     *             instance instead.
     */
    @Deprecated("Constructing devices with a CAN bus string is deprecated", ReplaceWith("TalonFX(deviceId, CANBus(canbus))"))
    constructor(deviceId: Int, canbus: String) : this(deviceId, CANBus(canbus))

    constructor(deviceId: Int, canbus: CANBus) : super(deviceId, canbus) {
        talonFXs.add(this)
        disconnectKey = "Connected/Talon/ID " + getDeviceID()
        tempKey = "Temperature/Talon/ID " + getDeviceID()

        val firmwareVersion = getVersion().getValue().toInt()
        val connected = isConnected()
        Logger.recordOutput(disconnectKey, connected)
        Logger.recordOutput("Firmware/Talon/ID " + getDeviceID(), firmwareVersion)
        if (connected && firmwareVersion != Constants.TALONFX_TARGET_FIRMWARE) {
            appendId(motorsWithIncorrectFirmwareVersion, null, getDeviceID())
        }
    }

    /**
     * check if the motor is overheating, and update alert if so
     */
    private fun temperatureCheck() {
        val tempC = getDeviceTemp().getValueAsDouble()
        Logger.recordOutput(tempKey, tempC)

        if (tempC > Constants.MAX_MOTOR_TEMP.`in`(Celsius)) {
            if (java.lang.Double.valueOf(getDeviceTemp().getValueAsDouble()) != overheatedMotorIds[getDeviceID()]) {
                overheatHasChanged = true
                overheatedMotorIds[getDeviceID()] = getDeviceTemp().getValueAsDouble()
            }
        } else {
            if (overheatedMotorIds.containsKey(getDeviceID())) {
                overheatedMotorIds.remove(getDeviceID())
                overheatHasChanged = true
            }
        }
    }

    /**
     * check if the motor is disconnected, and update alert if so
     */
    private fun disconnectCheck() {
        val disconnected = !isConnected()
        Logger.recordOutput(disconnectKey, !disconnected)
        if (disconnected) {
            if (disconnectedMotorIds.add(getDeviceID())) {
                disconnectHasChanged = true
            }
        } else {
            if (disconnectedMotorIds.remove(getDeviceID())) {
                disconnectHasChanged = true
            }
        }
    }

    companion object {
        private val motorsWithIncorrectFirmwareVersion = StringBuilder()
        private val motorsWithIncorrectFirmwareVersionAlert = Alert("Firmware version mismatch on TalonFXs: ", AlertType.kWarning)

        private val disconnectedMotorIds: MutableSet<Int> = HashSet()
        private var disconnectHasChanged = true
        private val disconnectedMotors = StringBuilder()
        private val motorDisconnectAlert = Alert("TalonFXs are disconnected: ", AlertType.kError)

        private val overheatedMotors = StringBuilder()
        private val overheatedMotorIds: MutableMap<Int, Double> = HashMap()
        private var overheatHasChanged = true
        private val motorOverheatAlert = Alert("TalonFX connected motor overheat: ", AlertType.kWarning)

        private val talonFXs: MutableSet<TalonFX> = LinkedHashSet()

        @JvmStatic
        fun getFirmwareAlert(): Alert = motorsWithIncorrectFirmwareVersionAlert

        @JvmStatic
        fun getDisconnectedAlert(): Alert = motorDisconnectAlert

        /**
         * call after all sparks have been initialized
         */
        @JvmStatic
        fun initAlerts() {
            if (motorsWithIncorrectFirmwareVersion.isNotEmpty()) {
                getFirmwareAlert().setText("Firmware version mismatch on TalonFXs: $motorsWithIncorrectFirmwareVersion")
                getFirmwareAlert().set(true)
            }
            if (disconnectedMotors.isNotEmpty()) {
                getDisconnectedAlert().setText("TalonFXes are disconnected: $disconnectedMotors")
                getDisconnectedAlert().set(true)
            }
        }

        @JvmStatic
        fun periodic() {
            for (talon in talonFXs) {
                talon.disconnectCheck()
                talon.temperatureCheck()
            }
            if (overheatHasChanged) {
                updateOverheatText()
                motorOverheatAlert.setText("TalonFX connected motor overheat: $overheatedMotors")
                overheatHasChanged = false
            }
            if (disconnectHasChanged) {
                updateDisconnectText()
                motorDisconnectAlert.setText("TalonFXes are disconnected: $disconnectedMotors")
                disconnectHasChanged = false
            }
        }

        private fun updateOverheatText() {
            overheatedMotors.setLength(0)
            if (overheatedMotorIds.size <= 0) {
                return
            }
            overheatedMotors.append("TalonFX connected motor overheat: ")

            val ids = overheatedMotorIds.keys.toTypedArray()
            overheatedMotors.append(formatOverheatEntry(ids[0], overheatedMotorIds[ids[0]]!!))

            for (i in 1 until ids.size) {
                overheatedMotors.append(", " + formatOverheatEntry(ids[i], overheatedMotorIds[ids[i]]!!))
            }
        }

        private fun updateDisconnectText() {
            disconnectedMotors.setLength(0)
            if (disconnectedMotorIds.size <= 0) {
                return
            }
            disconnectedMotors.append("TalonFXes are disconnected: ")

            val ids = disconnectedMotorIds.toTypedArray()
            disconnectedMotors.append("ID " + ids[0])

            for (i in 1 until ids.size) {
                disconnectedMotors.append(", ID " + ids[i])
            }
        }

        private fun formatOverheatEntry(id: Int, tempC: Double): String {
            return String.format(Locale.US, "ID %d(%.1fC)", id, tempC)
        }

        private fun appendId(builder: StringBuilder, seenIds: MutableSet<Int>?, id: Int) {
            if (seenIds != null && !seenIds.add(id)) {
                return
            }
            if (builder.isNotEmpty()) {
                builder.append(", ")
            }
            builder.append(id)
        }
    }
}
