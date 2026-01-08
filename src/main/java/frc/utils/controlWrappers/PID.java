package frc.utils.controlWrappers;

import edu.wpi.first.math.controller.PIDController;


/*
 * wrapper for PIDController that allows setting gains from a record
 */
public class PID extends PIDController{
    public PID(double kP, double kI, double kD){
        super(kP, kI, kD);
    }
    public PID(double kP, double kI, double kD, double period){
        super(kP, kI, kD, period);
    }
    public PID(PIDGains.PID gains){
        super(gains.kP(), gains.kI(), gains.kD());
    }
}
