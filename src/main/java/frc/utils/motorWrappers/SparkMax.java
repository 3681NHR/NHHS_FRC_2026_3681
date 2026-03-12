package frc.utils.motorWrappers;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

import static edu.wpi.first.units.Units.Celsius;

public class SparkMax extends com.revrobotics.spark.SparkMax {
    private static final StringBuilder motorsWithIncorrectFirmwareVersion = new StringBuilder();
    private static final Alert motorsWithIncorrectFirmwareVersionAlert = new Alert("Firmware version mismatch on SparkMaxes: ", AlertType.kWarning);

    private static final Set<Integer> disconnectedMotorIds = new HashSet<>();
    private static boolean disconnectHasChanged = true;
    private static final StringBuilder disconnectedMotors = new StringBuilder();
    private static final Alert motorDisconnectAlert = new Alert("SparkMaxes are disconnected: ", AlertType.kError);
    
    private static final StringBuilder overheatedMotors = new StringBuilder();
    private static final Map<Integer, Double> overheatedMotorIds = new HashMap<>();
    private static boolean overheatHasChanged = true;
    private static final Alert motorOverheatAlert = new Alert("SparkMax connected motor overheat: ", AlertType.kWarning);
    
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
        if (connected && firmwareVersion != Constants.SPARKMAX_TARGET_FIRMWARE) {
            appendId(motorsWithIncorrectFirmwareVersion, null, getDeviceId());
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
            getFirmwareAlert().setText("Firmware version mismatch on SparkMaxes: " + motorsWithIncorrectFirmwareVersion);
            getFirmwareAlert().set(true);
        }
        if (!disconnectedMotors.isEmpty()) {
            getDisconnectedAlert().setText("SparkMaxes are disconnected: " + disconnectedMotors);
            getDisconnectedAlert().set(true);
        }
    }

    public static void periodic() {
        for (SparkMax sparkMax : sparkMaxes) {
            sparkMax.disconnectCheck();
            sparkMax.temperatureCheck();
        }
        if(overheatHasChanged){
            updateOverheatText();
            motorOverheatAlert.setText("SparkMax connected motor overheat: " + overheatedMotors);
            overheatHasChanged = false;
        }
        if(disconnectHasChanged){
            updateDisconnectText();
            motorDisconnectAlert.setText("SparkMaxes are disconnected: " + disconnectedMotors);
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
        overheatedMotors.append("SparkMax connected motor overheat: ");

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
        disconnectedMotors.append("SparkMaxes are disconnected: ");

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
        double tempC = getMotorTemperature();
        Logger.recordOutput(tempKey, tempC);

        if (tempC > Constants.MAX_MOTOR_TEMP.in(Celsius)) {
            if(!Double.valueOf(getMotorTemperature()).equals(overheatedMotorIds.get(getDeviceId()))){
                overheatHasChanged = true;
                overheatedMotorIds.put(getDeviceId(), getMotorTemperature());
            }
        } else {
            if(overheatedMotorIds.containsKey(getDeviceId())){
                overheatedMotorIds.remove(getDeviceId());
                overheatHasChanged = true;
            }
        }
    }

    /**
     * check if the motor is disconnected, and update alert if so
     */
    private void disconnectCheck() {
        boolean disconnected = getLastError() == REVLibError.kCANDisconnected;
        Logger.recordOutput(disconnectKey, !disconnected);
        
        if (disconnected) {
            if (disconnectedMotorIds.add(getDeviceId())) {
                disconnectHasChanged = true;
            }
        } else {
            if(disconnectedMotorIds.remove(getDeviceId())){
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
