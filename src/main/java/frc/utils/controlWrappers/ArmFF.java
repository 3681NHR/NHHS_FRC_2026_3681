package frc.utils.controlWrappers;

import edu.wpi.first.math.controller.ArmFeedforward;

/*
 * wrapper for ArmFeedforward that allows setting gains from a record
 */
public class ArmFF extends ArmFeedforward{
    public ArmFF(double kS, double kG, double kV, double kA){
        super(kS, kG, kV, kA);
    }
    public ArmFF(double kS, double kG, double kV, double kA, double dt){
        super(kS, kG, kV, kA, dt);
    }
    public ArmFF(PIDGains.GravityFF gains){
        super(gains.kS(), gains.kG(), gains.kV(), gains.kA());
    }
}
