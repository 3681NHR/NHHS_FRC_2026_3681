package frc.utils.motorWrappers

import com.revrobotics.REVLibError
import com.revrobotics.spark.SparkMax as RevSparkMax
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import frc.robot.constants.Constants
import org.littletonrobotics.junction.Logger
import java.util.HashMap
import java.util.HashSet
import java.util.LinkedHashSet
import java.util.Locale
import java.util.Map
import java.util.Set

class SparkMax(deviceId: Int, type: MotorType) : RevSparkMax(deviceId, type) {

    private val disconnectKey: String
    private val tempKey: String

    init {
        sparkMaxes.add(this)
        disconnectKey = "Connected/Spark/ID " + getDeviceId()
        tempKey = "Temperature/Spark/ID " + getDeviceId()

        val firmwareVersion = getFirmwareVersion()
        val connected = getLastError() != REVLibError.kCANDisconnected
        Logger.recordOutput(disconnectKey, connected)
        Logger.recordOutput("Firmware/Spark/ID " + getDeviceId(), firmwareVersion)
        if (connected && firmwareVersion != Constants.SPARKMAX_TARGET_FIRMWARE) {
            appendId(motorsWithIncorrectFirmwareVersion, null, getDeviceId())
        }
    }

    /**
     * check if the motor is overheating, and update alert if so
     */
    private fun temperatureCheck() {
        val tempC = getMotorTemperature()
        Logger.recordOutput(tempKey, tempC)

        if (tempC > Constants.MAX_MOTOR_TEMP.`in`(Celsius)) {
            if (java.lang.Double.valueOf(getMotorTemperature()) != overheatedMotorIds[getDeviceId()]) {
                overheatHasChanged = true
                overheatedMotorIds[getDeviceId()] = getMotorTemperature()
            }
        } else {
            if (overheatedMotorIds.containsKey(getDeviceId())) {
                overheatedMotorIds.remove(getDeviceId())
                overheatHasChanged = true
            }
        }
    }

    /**
     * check if the motor is disconnected, and update alert if so
     */
    private fun disconnectCheck() {
        val disconnected = getLastError() == REVLibError.kCANDisconnected
        Logger.recordOutput(disconnectKey, !disconnected)

        if (disconnected) {
            if (disconnectedMotorIds.add(getDeviceId())) {
                disconnectHasChanged = true
            }
        } else {
            if (disconnectedMotorIds.remove(getDeviceId())) {
                disconnectHasChanged = true
            }
        }
    }

    companion object {
        private val motorsWithIncorrectFirmwareVersion = StringBuilder()
        private val motorsWithIncorrectFirmwareVersionAlert = Alert("Firmware version mismatch on SparkMaxes: ", AlertType.kWarning)

        private val disconnectedMotorIds: MutableSet<Int> = HashSet()
        private var disconnectHasChanged = true
        private val disconnectedMotors = StringBuilder()
        private val motorDisconnectAlert = Alert("SparkMaxes are disconnected: ", AlertType.kError)

        private val overheatedMotors = StringBuilder()
        private val overheatedMotorIds: MutableMap<Int, Double> = HashMap()
        private var overheatHasChanged = true
        private val motorOverheatAlert = Alert("SparkMax connected motor overheat: ", AlertType.kWarning)

        private val sparkMaxes: MutableSet<SparkMax> = LinkedHashSet()

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
                getFirmwareAlert().setText("Firmware version mismatch on SparkMaxes: $motorsWithIncorrectFirmwareVersion")
                getFirmwareAlert().set(true)
            }
            if (disconnectedMotors.isNotEmpty()) {
                getDisconnectedAlert().setText("SparkMaxes are disconnected: $disconnectedMotors")
                getDisconnectedAlert().set(true)
            }
        }

        @JvmStatic
        fun periodic() {
            for (sparkMax in sparkMaxes) {
                sparkMax.disconnectCheck()
                sparkMax.temperatureCheck()
            }
            if (overheatHasChanged) {
                updateOverheatText()
                motorOverheatAlert.setText("SparkMax connected motor overheat: $overheatedMotors")
                overheatHasChanged = false
            }
            if (disconnectHasChanged) {
                updateDisconnectText()
                motorDisconnectAlert.setText("SparkMaxes are disconnected: $disconnectedMotors")
                disconnectHasChanged = false
            }
        }

        private fun updateOverheatText() {
            overheatedMotors.setLength(0)
            if (overheatedMotorIds.size <= 0) {
                return
            }
            overheatedMotors.append("SparkMax connected motor overheat: ")

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
            disconnectedMotors.append("SparkMaxes are disconnected: ")

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
