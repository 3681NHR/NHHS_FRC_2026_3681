package frc.utils.controlWrappers;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/*
 * wrapper for ProfiledPIDController that allows setting gains from a record
 */
public class ProfiledPID extends ProfiledPIDController{
    public ProfiledPID(double kP, double kI, double kD, Constraints constraints){
        super(kP, kI, kD, constraints);
    }
    public ProfiledPID(double kP, double kI, double kD, Constraints constraints, double period){
        super(kP, kI, kD, constraints, period);
    }
    public ProfiledPID(PIDGains.ProfiledPID gains){
        super(gains.kP, gains.kI, gains.kD, new Constraints(gains.maxSpeed, gains.maxAccel));
    }

    public void setSpeed(double maxSpeed){
        super.setConstraints(new Constraints(maxSpeed, super.getConstraints().maxAcceleration));
    }
    public void setAccel(double maxAccel){
        super.setConstraints(new Constraints(super.getConstraints().maxVelocity, maxAccel));
    }
    public double getMaxSpeed(){
        return super.getConstraints().maxVelocity;
    }
    public double getMaxAccel(){
        return super.getConstraints().maxAcceleration;
    }
    public void setGains(PIDGains.ProfiledPID gains){
        setPID(gains.kP, gains.kI, gains.kD);
        setConstraints(new Constraints(gains.maxSpeed, gains.maxAccel));
    }
}
