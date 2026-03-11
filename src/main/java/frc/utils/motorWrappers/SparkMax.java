package frc.utils.motorWrappers;

import com.revrobotics.REVLibError;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.Constants;

public class SparkMax extends com.revrobotics.spark.SparkMax {
  private static String motorsWithIncorrectFirmwareVersion = "";
  private static final Alert motorsWithIncorrectFirmwareVersionAlert = new Alert("Firmware version mismatch on SparkMaxes: ", AlertType.kWarning);
  private static String motorsThatAreDisconnected = "";
  private static final Alert motorsThatAreDisconnectedAlert = new Alert("SparkMaxes that are disconnected: ", AlertType.kError);
  /**
   * Create a new object to control a SPARK MAX motor Controller
   *
   * @param deviceId The device ID.
   * @param type The motor type connected to the controller. Brushless motor wires must be connected
   *     to their matching colors and the hall sensor must be plugged in. Brushed motors must be
   *     connected to the Red and Black terminals only.
   */
  public SparkMax(int deviceId, MotorType type) {
    super(deviceId, type);
    boolean connected = this.getLastError() != REVLibError.kCANDisconnected;
    Logger.recordOutput("Connected/Spark/" + this.getDeviceId(), connected);
    Logger.recordOutput("Firmware/Spark/" + this.getDeviceId() + " firmware version", this.getFirmwareVersion());
    if (!connected) {
      motorsThatAreDisconnected += (motorsThatAreDisconnected.isBlank() ? "" : ", ") + getDeviceId();
      motorsThatAreDisconnectedAlert.setText("SparkMaxes that are disconnected: " + motorsThatAreDisconnected);
      if (!motorsThatAreDisconnectedAlert.get()) {
        motorsThatAreDisconnectedAlert.set(true);
      }
    }
    else if (!DriverStation.isFMSAttached() && this.getFirmwareVersion() != Constants.SPARKMAX_TARGET_FIRMWARE) {
      motorsWithIncorrectFirmwareVersion += (motorsWithIncorrectFirmwareVersion.isBlank() ? "" : ", ") + getDeviceId();
      motorsWithIncorrectFirmwareVersionAlert.setText("Firmware version mismatch on SparkMaxes: " + motorsWithIncorrectFirmwareVersion);
      if (!motorsWithIncorrectFirmwareVersionAlert.get()) {
        motorsWithIncorrectFirmwareVersionAlert.set(true);
      }
    }
  }
  public static Alert getFirmwareAlert() {
    return motorsWithIncorrectFirmwareVersionAlert;
  }
  public static Alert getDisconnectedAlert() {
    return motorsThatAreDisconnectedAlert;
  }
}
