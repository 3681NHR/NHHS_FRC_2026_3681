package frc.utils.motorWrappers;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import java.util.*;

import static edu.wpi.first.units.Units.Celsius;

/**
 * # TalonFX motor wrapper
 *
 * this class extends the base talonFX motor controller with automated alerts for disconnects, faults, and temp monitoring
 */
public class TalonFX extends com.ctre.phoenix6.hardware.TalonFX {
    private static final List<TalonFX> talonFXs = new ArrayList<>();

    private final Alert disconnectAlert = new Alert("", AlertType.kError);
    private final Alert tempWarning = new Alert("", AlertType.kWarning);
    private final Alert hardwareFaultAlert = new Alert("", AlertType.kError);
    private final Alert tempFaultAlert = new Alert("", AlertType.kError);
    private final Alert voltageFaultAlert = new Alert("", AlertType.kError);
    private final Alert licenseFaultAlert = new Alert("", AlertType.kError);
    private final Alert remoteSensorFaultAlert = new Alert("", AlertType.kError);

    private String name = null;
    private Temperature warningTemp = Celsius.of(80);//C
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

        this.withName("TalonFX ID:" + deviceId);
    }

    public TalonFX(int deviceId, CANBus canbus) {
        super(deviceId, canbus);
        talonFXs.add(this);
    }

    /**
     * set the name of the controller, this will show in all alerts, the default name is 'TalonFX ID:[CAN id]'
     * @param name the new name to be used
     * @return a reference to itself for chaining
     */
    public TalonFX withName(String name){
        this.name = name;
        disconnectAlert       .setText(name + " is disconnected!");
        tempWarning           .setText(name + " has a high temp!");
        hardwareFaultAlert    .setText(name + " hardware fault!");
        tempFaultAlert        .setText(name + " overheat!");
        voltageFaultAlert     .setText(name + " voltage fault!");
        licenseFaultAlert     .setText(name + " is unlicensed!");
        remoteSensorFaultAlert.setText(name + " remote sensor fault!");
        return this;
    }

    /**
     * sets the threshold that a temp warning will show, does not affect thermal protection on the motor
     * @param temp
     * @return
     */
    public TalonFX withWarningTemp(Temperature temp){
        this.warningTemp = temp;
        return this;
    }

    public static void periodic() {
        for (TalonFX talon : talonFXs) {
            talon.updateAlerts();
        }
    }
    public void updateAlerts(){
        disconnectAlert.set(!isConnected());
        var temp = getDeviceTemp().getValue();

        tempWarning.setText(name + " has a high temp!("+temp.in(Celsius)+"C)");
        tempWarning.set(temp.gte(warningTemp));

        hardwareFaultAlert.set(
                getFault_BootDuringEnable().getValue()
                || getFault_Hardware().getValue()
                || getFault_StaticBrakeDisabled().getValue()
        );

        tempFaultAlert.set(
                getFault_ProcTemp().getValue()
                || getFault_DeviceTemp().getValue()
        );

        voltageFaultAlert.set(
                getFault_OverSupplyV().getValue()
                || getFault_UnstableSupplyV().getValue()
                || getFault_Undervoltage().getValue()
                || getStickyFault_BridgeBrownout().getValue()
        );

        licenseFaultAlert.set(
                getFault_UnlicensedFeatureInUse().getValue()
                || getFault_UsingFusedCANcoderWhileUnlicensed().getValue()
        );

        remoteSensorFaultAlert.set(
                getFault_MissingDifferentialFX().getValue()
                || getFault_MissingHardLimitRemote().getValue()
                || getFault_MissingSoftLimitRemote().getValue()
                || getFault_RemoteSensorPosOverflow().getValue()
                || getFault_RemoteSensorReset().getValue()
                || getFault_RemoteSensorDataInvalid().getValue()
                || getFault_FusedSensorOutOfSync().getValue()
        );
    }

}
