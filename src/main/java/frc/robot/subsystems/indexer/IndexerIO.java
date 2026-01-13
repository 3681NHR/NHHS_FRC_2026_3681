package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    
    public void updateInputs(IndexerIOInputs input);
    
    @AutoLog
    public class IndexerIOInputs{

    }
}
