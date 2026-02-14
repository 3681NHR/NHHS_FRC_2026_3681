package frc.utils.controlWrappers;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/*
 * wrapper for SimpleMotorFeedforward that allows setting gains from a record
 */
public class SimpleFF extends SimpleMotorFeedforward{
    public SimpleFF(double kS, double kV, double kA){
        super(kS, kV, kA);
    }
    public SimpleFF(double kS, double kV, double kA, double dt){
        super(kS, kV, kA, dt);
    }
    public SimpleFF(PIDGains.SimpleFF gains){
        super(gains.kS, gains.kV, gains.kA);
    }
}
