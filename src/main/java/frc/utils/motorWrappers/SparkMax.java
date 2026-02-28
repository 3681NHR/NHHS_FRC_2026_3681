package frc.utils.motorWrappers;

import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;

public class SparkMax extends com.revrobotics.spark.SparkMax {
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
    if (this.getFirmwareString() != Constants.SPARKMAX_TARGET_FIRMWARE) {
        RobotContainer.alert("SPARK id " + deviceId + " is version " + this.getFirmwareString() + ", expected " + Constants.SPARKMAX_TARGET_FIRMWARE, AlertType.kWarning);
    }
  }
    
}
