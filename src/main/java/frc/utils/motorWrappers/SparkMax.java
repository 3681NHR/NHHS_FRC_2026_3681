package frc.utils.motorWrappers;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Locale;
import java.util.Set;

import static edu.wpi.first.units.Units.Celsius;

public class SparkMax extends com.revrobotics.spark.SparkMax {
    private static final StringBuilder motorsWithIncorrectFirmwareVersion = new StringBuilder();
    private static final Alert motorsWithIncorrectFirmwareVersionAlert = new Alert("Firmware version mismatch on SparkMaxes: ", AlertType.kWarning);
    private static final StringBuilder motorsThatAreDisconnected = new StringBuilder();
    private static final Alert motorsThatAreDisconnectedAlert = new Alert("SparkMaxes are disconnected: ", AlertType.kError);
    private static final Alert motorOverheatAlert = new Alert("SparkMax overheat: ", AlertType.kWarning);
    private static final Set<Integer> disconnectedMotorIds = new HashSet<>();
    private static final Set<SparkMax> sparkMaxes = new LinkedHashSet<>();

    private final String disconnectKey;
    private final String tempKey;

    /**
     * Create a new object to control a SPARK MAX motor Controller
     *
     * @param deviceId The device ID.
     * @param type     The motor type connected to the controller. Brushless motor wires must be connected
     *                 to their matching colors and the hall sensor must be plugged in. Brushed motors must be
     *                 connected to the Red and Black terminals only.
     */
    public SparkMax(int deviceId, MotorType type) {
        super(deviceId, type);
        sparkMaxes.add(this);
        disconnectKey = "Connected/Spark/ID " + getDeviceId();
        tempKey = "Temperature/Spark/ID " + getDeviceId();

        int firmwareVersion = getFirmwareVersion();
        boolean connected = getLastError() != REVLibError.kCANDisconnected;
        Logger.recordOutput(disconnectKey, connected);
        Logger.recordOutput("Firmware/Spark/ID " + getDeviceId(), firmwareVersion);
        if (!connected) {
            appendId(motorsThatAreDisconnected, disconnectedMotorIds, getDeviceId());
        } else if (!DriverStation.isFMSAttached() && firmwareVersion != Constants.SPARKMAX_TARGET_FIRMWARE) {
            appendId(motorsWithIncorrectFirmwareVersion, null, getDeviceId());
        }
    }

    public static Alert getFirmwareAlert() {
        return motorsWithIncorrectFirmwareVersionAlert;
    }

    public static Alert getDisconnectedAlert() {
        return motorsThatAreDisconnectedAlert;
    }

    /**
     * call after all sparks have been initialized
     */
    public static void initAlerts() {
        if (!motorsWithIncorrectFirmwareVersion.isEmpty()) {
            getFirmwareAlert().setText("Firmware version mismatch on SparkMaxes: " + motorsWithIncorrectFirmwareVersion);
            getFirmwareAlert().set(true);
        }
        if (!motorsThatAreDisconnected.isEmpty()) {
            getDisconnectedAlert().setText("SparkMaxes that are disconnected: " + motorsThatAreDisconnected);
            getDisconnectedAlert().set(true);
        }
    }

    public static void periodic() {
        StringBuilder overheatingMotors = new StringBuilder();
        for (SparkMax sparkMax : sparkMaxes) {
            sparkMax.disconnectCheck();
            sparkMax.temperatureCheck(overheatingMotors);
        }
        if (!overheatingMotors.isEmpty()) {
            motorOverheatAlert.setText("SparkMax overheat: " + overheatingMotors);
            motorOverheatAlert.set(true);
        } else {
            motorOverheatAlert.set(false);
        }
    }

    private void temperatureCheck(StringBuilder overheatingMotors) {
        double tempC = getMotorTemperature();
        Logger.recordOutput(tempKey, tempC);

        if (tempC > Constants.MAX_MOTOR_TEMP.in(Celsius)) {
            if (!overheatingMotors.isEmpty()) {
                overheatingMotors.append(", ");
            }
            overheatingMotors.append(formatOverheatEntry(getDeviceId(), tempC));
        }
    }

    private void disconnectCheck() {
        boolean connected = getLastError() != REVLibError.kCANDisconnected;
        Logger.recordOutput(disconnectKey, connected);
        if (!connected) {
            if (disconnectedMotorIds.add(getDeviceId())) {
                appendId(motorsThatAreDisconnected, null, getDeviceId());
                motorsThatAreDisconnectedAlert.setText("SparkMaxes that are disconnected: " + motorsThatAreDisconnected);
            }
            motorsThatAreDisconnectedAlert.set(true);
        }
    }

    private static void appendId(StringBuilder builder, Set<Integer> seenIds, int id) {
        if (seenIds != null && !seenIds.add(id)) {
            return;
        }
        if (!builder.isEmpty()) {
            builder.append(", ");
        }
        builder.append(id);
    }

    private static String formatOverheatEntry(int id, double tempC) {
        return String.format(Locale.US, "ID %d(%.1fC)", id, tempC);
    }
}
