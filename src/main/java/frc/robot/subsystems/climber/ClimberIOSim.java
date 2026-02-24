package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ClimbConstants.CLIMB_PID_GAINS;
import static frc.robot.constants.ClimbConstants.FF;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.Constants;
import frc.utils.controlWrappers.ElevatorFF;
import frc.utils.controlWrappers.ProfiledPID;

public class ClimberIOSim implements ClimberIO {
    // TODO: tune sim params to be realistic
    private final ElevatorSim sim = new ElevatorSim(
            DCMotor.getNEO(1),
            9.0,
            2.5,
            0.02,
            0.0,
            2.0,
            true,
            0.0);

    private final ProfiledPID pid = new ProfiledPID(CLIMB_PID_GAINS);
    private final ElevatorFF ff = new ElevatorFF(FF);

    private boolean openLoop = false;
    private double goalMeters = 0.0;
    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(ClimberIOInputs input){
        if (!openLoop) {
            appliedVolts = pid.calculate(sim.getPositionMeters(), goalMeters) + ff.calculate(pid.getSetpoint().velocity);
        }

        appliedVolts = MathUtil.clamp(appliedVolts, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
        sim.setInputVoltage(appliedVolts);
        sim.update(Constants.EVENT_LOOP_TIME);

        input.motorVoltageOut = Volts.of(appliedVolts);
        input.motorCurrentOut = Amps.of(Math.abs(sim.getCurrentDrawAmps()));
        input.motorTemp = Celsius.of(25.0);
        input.encoderPosition = Meters.of(sim.getPositionMeters());
        input.encoderVelocity = MetersPerSecond.of(sim.getVelocityMetersPerSecond());
        input.climbVelocitySetpoint = MetersPerSecond.of(pid.getSetpoint().velocity);
        input.climbPositionSetpoint = Meters.of(pid.getSetpoint().position);
        input.connected = true;
        input.goal = Meters.of(goalMeters);
        input.atSetpoint = pid.atSetpoint();
        input.openLoop = openLoop;
    }

    /** sets the goal position for the climber. */
    @Override
    public void setSetpoint(double position){
        openLoop = false;
        goalMeters = position;
    }

    @Override
    public void zeroEncoder(){
        sim.setState(0.0, sim.getVelocityMetersPerSecond());
        goalMeters = 0.0;
        pid.reset(0.0);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        openLoop = true;
        appliedVolts = voltage.in(Volts);
    }
}
