package frc.utils.controlWrappers;

import edu.wpi.first.math.controller.ElevatorFeedforward;

/*
 * wrapper for ElevatorFeedforward that allows setting gains from a record
 */
public class ElevatorFF extends ElevatorFeedforward{
    public ElevatorFF(double kS, double kG, double kV, double kA){
        super(kS, kG, kV, kA);
    }
    public ElevatorFF(double kS, double kG, double kV, double kA, double dt){
        super(kS, kG, kV, kA, dt);
    }
    public ElevatorFF(PIDGains.GravityFF gains){
        super(gains.kS(), gains.kG(), gains.kV(), gains.kA());
    }
}
