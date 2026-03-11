package frc.utils.motorWrappers;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Locale;
import java.util.Set;

import static edu.wpi.first.units.Units.Celsius;

public class TalonFX extends com.ctre.phoenix6.hardware.TalonFX {

    private static final StringBuilder motorsWithIncorrectFirmwareVersion = new StringBuilder();
    private static final Alert motorsWithIncorrectFirmwareVersionAlert = new Alert(
            "Firmware version mismatch on Talons: ", AlertType.kWarning);
    private static final StringBuilder motorsThatAreDisconnected = new StringBuilder();
    private static final Alert motorsThatAreDisconnectedAlert = new Alert(
            "Talon FXs are disconnected: ", AlertType.kError);
    private static final Alert motorOverheatAlert = new Alert("TalonFX overheat: ", AlertType.kWarning);
    private static final Set<Integer> disconnectedMotorIds = new HashSet<>();
    private static final Set<TalonFX> talonFXs = new LinkedHashSet<>();

    private final String disconnectKey;
    private final String tempKey;

    /**
     * Constructs a new Talon FX motor controller object.
     * <p>
     * Constructs the device using the default CAN bus for the system
     * (see {@link CANBus#CANBus()}).
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner
     */
    public TalonFX(int deviceId) {
        this(deviceId, new CANBus());
    }

    /**
     * Constructs a new Talon FX motor controller object.
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner
     * @param canbus   Name of the CAN bus this device is on. Possible CAN bus
     *                 strings are:
     *                 <ul>
     *                 <li>"rio" for the native roboRIO CAN bus
     *                 <li>CANivore name or serial number
     *                 <li>SocketCAN interface (non-FRC Linux only)
     *                 <li>"*" for any CANivore seen by the program
     *                 <li>empty string (default) to select the default for the
     *                 system:
     *                 <ul>
     *                 <li>"rio" on roboRIO
     *                 <li>"can0" on Linux
     *                 <li>"*" on Windows
     *                 </ul>
     *                 </ul>
     *
     * @deprecated Constructing devices with a CAN bus string is deprecated for
     *             removal
     *             in the 2027 season. Construct devices using a {@link CANBus}
     *             instance instead.
     */
    @Deprecated(since = "2026", forRemoval = true)
    public TalonFX(int deviceId, String canbus) {
        this(deviceId, new CANBus(canbus));
    }

    public TalonFX(int deviceId, CANBus canbus) {
        super(deviceId, canbus);
        talonFXs.add(this);
        disconnectKey = "Connected/Talon/ID " + getDeviceID();
        tempKey = "Temperature/Talon/ID " + getDeviceID();

        int firmwareVersion = getVersion().getValue();
        Logger.recordOutput(disconnectKey, isConnected());
        Logger.recordOutput("Firmware/Talon/ID " + getDeviceID(), firmwareVersion);
        if (!isConnected()) {
            appendId(motorsThatAreDisconnected, disconnectedMotorIds, getDeviceID());
        } else if (!DriverStation.isFMSAttached() && firmwareVersion != Constants.TALONFX_TARGET_FIRMWARE) {
            appendId(motorsWithIncorrectFirmwareVersion, null, getDeviceID());
        }
    }

    public static Alert getFirmwareAlert() {
        return motorsWithIncorrectFirmwareVersionAlert;
    }

    public static Alert getDisconnectedAlert() {
        return motorsThatAreDisconnectedAlert;
    }

    /**
     * call after all talons have been initialized
     */
    public static void initAlerts() {
        if (!motorsWithIncorrectFirmwareVersion.isEmpty()) {
            getFirmwareAlert().setText("Firmware version mismatch on Talons: " + motorsWithIncorrectFirmwareVersion);
            getFirmwareAlert().set(true);
        }
        if (!motorsThatAreDisconnected.isEmpty()) {
            getDisconnectedAlert().setText("Talon FXs that are disconnected: " + motorsThatAreDisconnected);
            getDisconnectedAlert().set(true);
        }
    }

    public static void periodic() {
        StringBuilder overheatingMotors = new StringBuilder();
        for (TalonFX talonFX : talonFXs) {
            talonFX.disconnectCheck();
            talonFX.temperatureCheck(overheatingMotors);
        }
        if (!overheatingMotors.isEmpty()) {
            motorOverheatAlert.setText("TalonFX overheat: " + overheatingMotors);
            motorOverheatAlert.set(true);
        } else {
            motorOverheatAlert.set(false);
        }
    }

    private void temperatureCheck(StringBuilder overheatingMotors) {
        double tempC = getDeviceTemp().getValue().in(Celsius);
        Logger.recordOutput(tempKey, tempC);

        if (tempC > Constants.MAX_MOTOR_TEMP.in(Celsius)) {
            if (!overheatingMotors.isEmpty()) {
                overheatingMotors.append(", ");
            }
            overheatingMotors.append(formatOverheatEntry(getDeviceID(), tempC));
        }
    }

    private void disconnectCheck() {
        boolean connected = isConnected();
        Logger.recordOutput(disconnectKey, connected);
        if (!connected) {
            if (disconnectedMotorIds.add(getDeviceID())) {
                appendId(motorsThatAreDisconnected, null, getDeviceID());
                motorsThatAreDisconnectedAlert.setText("Talon FXs that are disconnected: " + motorsThatAreDisconnected);
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
