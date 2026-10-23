package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.KickerConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {
	
	KickerIO io;
	KickerIOInputsAutoLogged in = new KickerIOInputsAutoLogged();
	
	private LoggedNetworkBoolean unloadEnabled = new LoggedNetworkBoolean("Overrides/Kicker unload", true);
    private final Alert sensorDisconnectAlert = new Alert("Kicker CANRange disconnected!", Alert.AlertType.kError);
	
	public Kicker(KickerIO io) {
		this.io = io;
	}
	
	@Override
	public void periodic() {
		io.updateInputs(in);
		Logger.processInputs("IO/Kicker", in);
		Logger.recordOutput("Subsystems/Kicker/state", (getCurrentCommand() == null ? "none" : getCurrentCommand().getName()));

        sensorDisconnectAlert.set(!in.sensorConnected);
	}
	
	public Command hold() {
		return Commands.run(() -> {
			if (unloadEnabled.get()){
                if(in.distance.lte(KICKER_UNLOAD_MAX_DISTANCE) && in.sensorConnected) {
				    io.setVout(KICKER_UNLOAD_VOLTAGE);
			    } else {
				    io.setVout(KICKER_UNLOAD_PARTIAL_VOLTAGE);//fuel may be in deadzone, prevents wasting power by running lower
                }
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
