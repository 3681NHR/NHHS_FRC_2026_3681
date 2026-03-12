package frc.utils.motorWrappers;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.Constants;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

import static edu.wpi.first.units.Units.Celsius;

public class TalonFX extends com.ctre.phoenix6.hardware.TalonFX {
    private static final StringBuilder motorsWithIncorrectFirmwareVersion = new StringBuilder();
    private static final Alert motorsWithIncorrectFirmwareVersionAlert = new Alert("Firmware version mismatch on TalonFXs: ", AlertType.kWarning);

    private static final Set<Integer> disconnectedMotorIds = new HashSet<>();
    private static boolean disconnectHasChanged = true;
    private static final StringBuilder disconnectedMotors = new StringBuilder();
    private static final Alert motorDisconnectAlert = new Alert("TalonFXs are disconnected: ", AlertType.kError);
    
    private static final StringBuilder overheatedMotors = new StringBuilder();
    private static final Map<Integer, Double> overheatedMotorIds = new HashMap<>();
    private static boolean overheatHasChanged = true;
    private static final Alert motorOverheatAlert = new Alert("TalonFX connected motor overheat: ", AlertType.kWarning);
    
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
        boolean connected = isConnected();
        Logger.recordOutput(disconnectKey, connected);
        Logger.recordOutput("Firmware/Talon/ID " + getDeviceID(), firmwareVersion);
        if (connected && firmwareVersion != Constants.SPARKMAX_TARGET_FIRMWARE) {
            appendId(motorsWithIncorrectFirmwareVersion, null, getDeviceID());
        }
    }

    public static Alert getFirmwareAlert() {
        return motorsWithIncorrectFirmwareVersionAlert;
    }

    public static Alert getDisconnectedAlert() {
        return motorDisconnectAlert;
    }

    /**
     * call after all sparks have been initialized
     */
    public static void initAlerts() {
        if (!motorsWithIncorrectFirmwareVersion.isEmpty()) {
            getFirmwareAlert().setText("Firmware version mismatch on TalonFXs: " + motorsWithIncorrectFirmwareVersion);
            getFirmwareAlert().set(true);
        }
        if (!disconnectedMotors.isEmpty()) {
            getDisconnectedAlert().setText("TalonFXes are disconnected: " + disconnectedMotors);
            getDisconnectedAlert().set(true);
        }
    }

    public static void periodic() {
        for (TalonFX talon : talonFXs) {
            talon.disconnectCheck();
            talon.temperatureCheck();
        }
        if(overheatHasChanged){
            updateOverheatText();
            motorOverheatAlert.setText("TalonFX connected motor overheat: " + overheatedMotors);
            overheatHasChanged = false;
        }
        if(disconnectHasChanged){
            updateDisconnectText();
            motorDisconnectAlert.setText("TalonFXes are disconnected: " + disconnectedMotors);
            disconnectHasChanged = false;
        }
    }

    private static void updateOverheatText(){
        //clear alert
        overheatedMotors.setLength(0);
        if(overheatedMotorIds.size() <= 0){
            //return if no overheat
            return;
        }
        overheatedMotors.append("TalonFX connected motor overheat: ");

        Integer[] ids = overheatedMotorIds.keySet().toArray(new Integer[0]);
        overheatedMotors.append(formatOverheatEntry(ids[0], overheatedMotorIds.get(ids[0])));

        for(int i=1; i<ids.length; i++){
            overheatedMotors.append(", " + formatOverheatEntry(ids[i], overheatedMotorIds.get(ids[i])));
        }
    }
    
    private static void updateDisconnectText(){
        disconnectedMotors.setLength(0);
        if(disconnectedMotorIds.size() <= 0){
            return;
        }
        disconnectedMotors.append("TalonFXes are disconnected: ");

        Integer[] ids = disconnectedMotorIds.toArray(new Integer[0]);
        disconnectedMotors.append("ID " + ids[0]);

        for(int i=1; i<ids.length; i++){
            disconnectedMotors.append(", ID " + ids[i]);
        }
    }

    /**
     * check if the motor is overheating, and update alert if so
     */
    private void temperatureCheck() {
        double tempC = getDeviceTemp().getValueAsDouble();
        Logger.recordOutput(tempKey, tempC);

        if (tempC > Constants.MAX_MOTOR_TEMP.in(Celsius)) {
            if(!Double.valueOf(getDeviceTemp().getValueAsDouble()).equals(overheatedMotorIds.get(getDeviceID()))){
                overheatHasChanged = true;
                overheatedMotorIds.put(getDeviceID(), getDeviceTemp().getValueAsDouble());
            }
        } else {
            if(overheatedMotorIds.containsKey(getDeviceID())){
                overheatedMotorIds.remove(getDeviceID());
                overheatHasChanged = true;
            }
        }
    }

    /**
     * check if the motor is disconnected, and update alert if so
     */
    private void disconnectCheck() {
        boolean disconnected = !isConnected();
        Logger.recordOutput(disconnectKey, !disconnected);
        if (disconnected) {
            if (disconnectedMotorIds.add(getDeviceID())) {
                disconnectHasChanged = true;
            }
        } else {
            if(disconnectedMotorIds.remove(getDeviceID())){
                disconnectHasChanged = true;
            }
        }
    }

    private static String formatOverheatEntry(int id, double tempC) {
        return String.format(Locale.US, "ID %d(%.1fC)", id, tempC);
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
}
