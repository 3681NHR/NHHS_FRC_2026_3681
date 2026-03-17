package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

@SuppressWarnings("PMD.UncommentedEmptyMethodBody")
public interface IndexerIO {

    default void updateInputs(IndexerIOInputs input) {}

    default void setVout(Voltage vout) {}
    
    @AutoLog
    class IndexerIOInputs{
        public AngularVelocity speed = RPM.zero();

        public Voltage motorVoltageOut = Volts.zero();
        public Current motorCurrentOut = Amps.zero();
        public Temperature motorTemp = Kelvin.zero().minus(Kelvin.one());

        public boolean motorConnected = false;
    }
}
