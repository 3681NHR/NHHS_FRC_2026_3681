package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.KickerConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {
	
	KickerIO io;
	KickerIOInputsAutoLogged in = new KickerIOInputsAutoLogged();
	
	private LoggedNetworkBoolean preloadEnabled = new LoggedNetworkBoolean("Overrides/Kicker Preload", false);
	
	public Kicker(KickerIO io) {
		this.io = io;
	}
	
	@Override
	public void periodic() {
		io.updateInputs(in);
		Logger.processInputs("IO/Kicker", in);
		Logger.recordOutput("Subsystems/Kicker/state", (getCurrentCommand() == null ? "none" : getCurrentCommand().getName()));
	}
	
	public Command hold() {
		return Commands.run(() -> {
			if (preloadEnabled.get() && 
				in.distance.lte(KICKER_PRELOAD_MAX_DISTANCE) && 
				in.hasBall &&
				in.sensorConnected) {

				io.setVout(KICKER_PRELOAD_VOLTAGE);
			} else {
				io.setVout(Volts.zero());
			}
		}, this).withName("Hold");
	}
	
	public Command feed() {
		return Commands.run(() -> {
			io.setVout(KICKER_FEED_VOLTAGE);
		}, this).withName("Feed");
	}
	public Command reverse() {
		return Commands.run(() -> {
			io.setVout(KICKER_FEED_VOLTAGE.unaryMinus());
		}, this).withName("Feed");
	}
	
	public Command voltageControl(Supplier<Voltage> volt) {
		return Commands.run(() -> {
			io.setVout(volt.get());
		}, this)
		.withName("Voltage Control");
	}
}
