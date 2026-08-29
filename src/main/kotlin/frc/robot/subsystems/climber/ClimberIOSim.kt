package frc.robot.subsystems.climber

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.LinearSystemSim
import frc.robot.constants.ClimberConstants.CLIMBER_HOME_ON_START
import frc.robot.constants.ClimberConstants.CLIMBER_ID_GAINS
import frc.robot.constants.ClimberConstants.CLIMBER_MAX_POSITION
import frc.robot.constants.ClimberConstants.CLIMBER_MIN_POSITION
import frc.robot.constants.ClimberConstants.CLIMBER_PID_GAINS
import frc.robot.constants.ClimberConstants.CLIMBER_FF_GAINS
import frc.robot.constants.Constants
import frc.utils.controlWrappers.ElevatorFF
import frc.utils.controlWrappers.ProfiledPID

class ClimberIOSim : ClimberIO {

    private val model: LinearSystem<N2, N1, N2> = LinearSystemId.identifyPositionSystem(CLIMBER_ID_GAINS.kV, CLIMBER_ID_GAINS.kA)
    private val sim = LinearSystemSim<N2, N1, N2>(model, 0.0, 0.001)

    private val pid = ProfiledPID(CLIMBER_PID_GAINS)
    private val ff = ElevatorFF(CLIMBER_FF_GAINS)

    private var openLoop = false
    private var goal: Distance = CLIMBER_MIN_POSITION
    private var appliedVolts: Voltage = Volts.zero()

    private var position: Distance = CLIMBER_MIN_POSITION
    private var offset: Distance = Meters.zero()

    private var homed: Boolean = CLIMBER_HOME_ON_START

    override fun updateInputs(input: ClimberIO.ClimberIOInputs) {
        sim.update(Constants.EVENT_LOOP_TIME)
        position = Meters.of(sim.output.get(0, 0)).plus(offset)

        if (!openLoop && homed) {
            appliedVolts = Volts.of(pid.calculate(position.`in`(Meters), goal.`in`(Meters)) + ff.calculate(pid.setpoint.velocity))
        }

        appliedVolts = Volts.of(
            MathUtil.clamp(appliedVolts.`in`(Volts), -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage())
        )

        val voltageInput: Double = if (DriverStation.isEnabled()) {
            appliedVolts.`in`(Volts) - (Math.min(CLIMBER_ID_GAINS.kS, Math.abs(appliedVolts.`in`(Volts))) * Math.signum(sim.output.get(1, 0))) // TODO add kg
        } else {
            -CLIMBER_ID_GAINS.kG
        }

        if (Meters.of(sim.output.get(0, 0)).gt(CLIMBER_MAX_POSITION)) {
            sim.setState(VecBuilder.fill(CLIMBER_MAX_POSITION.`in`(Meters), 0.0))
            sim.setInput(MathUtil.clamp(voltageInput, -12.0, 0.0))
        } else if (Meters.of(sim.output.get(0, 0)).lt(CLIMBER_MIN_POSITION)) {
            sim.setState(VecBuilder.fill(CLIMBER_MIN_POSITION.`in`(Meters), 0.0))
            sim.setInput(MathUtil.clamp(voltageInput, 0.0, 12.0))
        } else {
            sim.setInput(voltageInput)
        }

        input.position = position
        input.velocity = MetersPerSecond.of(sim.output.get(1, 0))

        input.motorVoltageOut = appliedVolts

        input.goal = goal
        input.atSetpoint = pid.atSetpoint()
        input.velocitySetpoint = MetersPerSecond.of(pid.setpoint.velocity)
        input.positionSetpoint = Meters.of(pid.setpoint.position)

        input.connected = true
        input.openLoop = openLoop
        input.homed = homed
    }

    /** sets the goal position for the climber. */
    override fun setGoal(goal: Distance) {
        openLoop = false
        this.goal = goal
    }

    override fun setVoltage(voltage: Voltage) {
        openLoop = true
        appliedVolts = voltage
    }

    override fun setHomed(homed: Boolean) {
        this.homed = homed
    }

    override fun setPosition(position: Distance) {
        offset = this.position.minus(offset).unaryMinus().plus(position)
    }
}
