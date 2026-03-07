package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.KickerConstants;

public class Kicker extends SubsystemBase {

  KickerIO io;
  KickerIOInputsAutoLogged in = new KickerIOInputsAutoLogged();

  private LoggedNetworkBoolean preloadEnabled = new LoggedNetworkBoolean("Controls/Kicker Preload", true);

  public Kicker(KickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(in);
    Logger.processInputs("IO/Kicker", in);
    Logger.recordOutput("Subsystems/Kicker/state",
        (getCurrentCommand() == null ? "none" : getCurrentCommand().getName()));
  }

  public Command hold() {
    return Commands.run(() -> {
      if (preloadEnabled.get()) {
        if (in.distance.lte(KickerConstants.PRELOAD_DISTANCE_THRESHOLD)) {
          io.setGoal(RPM.zero());
        } else if (in.distance.lte(KickerConstants.PRELOAD_MAX_DISTANCE)) {
          io.setGoal(KickerConstants.PRELOAD_VELOCITY);
        } else {
          io.setGoal(RPM.zero());
        }
      } else {
        io.setGoal(RPM.zero());
      }
    }, this).withName("Hold Ball");
  }

  public Command run() {
    return Commands.run(() -> {
      io.setGoal(KickerConstants.SHOOT_VELOCITY);
    }, this).withName("Run Kicker");
  }

  public Command velocityControl(Supplier<AngularVelocity> velocity) {
    return Commands.run(() -> {
      io.setGoal(velocity.get());
    }, this).withName("Velocity Control");
  }

  public Command voltageControl(Supplier<Voltage> volt) {
    return Commands.run(() -> {
      io.setVout(volt.get());
    }, this)
        .withName("Voltage Control");
  }
}
