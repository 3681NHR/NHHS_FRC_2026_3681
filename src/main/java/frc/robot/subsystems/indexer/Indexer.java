package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.IndexerConstants.INDEXER_FEED_VOLTAGE;

public class Indexer extends SubsystemBase {
	
	IndexerIO io;
	IndexerIOInputsAutoLogged in = new IndexerIOInputsAutoLogged();

	public Indexer(IndexerIO io) {
		this.io = io;
	}
	
	@Override
	public void periodic() {
		io.updateInputs(in);
		Logger.processInputs("IO/Indexer", in);
		Logger.recordOutput("Subsystems/Indexer/state", (getCurrentCommand() == null ? "none" : getCurrentCommand().getName()));
	}
	
	public Command feed() {
		return Commands.run(() -> {
			io.setVout(INDEXER_FEED_VOLTAGE);
		}, this).withName("Feed");
	}
	public Command reverse() {
		return Commands.run(() -> {
			io.setVout(INDEXER_FEED_VOLTAGE.unaryMinus());
		}, this).withName("Feed");
	}
    public Command stop() {
        return Commands.run(() -> {
            io.setVout(Volts.of(0));
        }, this).withName("Stop");
    }
	
	public Command voltageControl(Supplier<Voltage> volt) {
		return Commands.run(() -> {
			io.setVout(volt.get());
		}, this)
		.withName("Voltage Control");
	}
}
