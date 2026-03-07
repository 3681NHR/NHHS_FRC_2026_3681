package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;

public class IntakeIOReal implements IntakeIO {
    
    private SparkMax intakeMotor = new SparkMax(42, SparkMax.MotorType.kBrushless);

    public void updateInputs(IntakeIOInputs input){
        input.appliedVoltage = intakeMotor.getAppliedOutput();
    }
    public void setVout(double volt){
        intakeMotor.setVoltage(volt);
    }
}
