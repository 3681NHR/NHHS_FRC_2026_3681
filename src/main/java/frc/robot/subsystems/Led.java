package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.RobotMode;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.turret.Turret;

public class Led extends SubsystemBase {

    Launcher launcher;
    Hood hood;
    Turret turret;
    Drive drive;

    BooleanSupplier manualSupplier;

    private final AddressableLED led = new AddressableLED(0);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(70+5);
    private final AddressableLEDBufferView sideBuffer = new AddressableLEDBufferView(buffer, 70, 70+4);
    private final AddressableLEDBufferView turretBuffer = new AddressableLEDBufferView(buffer, 0, 70);

    private final LoggedNetworkBoolean rainbow = new LoggedNetworkBoolean("Overrides/LED Override", false);
    private final LoggedNetworkBoolean disable = new LoggedNetworkBoolean("Overrides/LED Disable", false);

    private final Debouncer readyDebouncer = new Debouncer(0.2);

    public Led(Launcher launcher,
    Hood hood,
    Turret turret,
    Drive drive,
    BooleanSupplier manualSupplier) {

        led.setLength(buffer.getLength());

        led.start();

        this.launcher =  launcher;
        this.hood =  hood;
        this.turret =  turret;
        this.drive =  drive;
        this.manualSupplier = manualSupplier;
    }

    @Override
    public void periodic() {

        Color turretState = readyDebouncer.calculate(turret.isReady() && launcher.isReady() && hood.isReady()) ? Color.kGreen : Color.kBlack;
        if(manualSupplier.getAsBoolean()){
            turretState = Color.kWhite;
        }
        if(!hood.isHomed()){
            turretState = Color.kRed;
        }
        if(hood.isHoming()){
            turretState = Color.kOrange;
        }
        Color sideState = drive.getFOD() ? new Color() : Color.kBlue;
        
        
        if(rainbow.get()){
            LEDPattern.rainbow(255, 255).applyTo(turretBuffer);
            LEDPattern.rainbow(255, 255).applyTo(sideBuffer);
        } else {
            LEDPattern.solid(turretState).atBrightness(Percent.of(50)).applyTo(turretBuffer);

            LEDPattern.solid(sideState).atBrightness(Percent.of(50)).applyTo(sideBuffer);
        }

        if(disable.get()){
            LEDPattern.solid(new Color()).applyTo(buffer);
        }

        led.setData(buffer);

        //only log all leds in replay, as toHexString() is slow
        if(Constants.MODE == RobotMode.REPLAY){
            String[] leds = new String[buffer.getLength()];
            for (int i = 0; i < buffer.getLength(); i++) {
                leds[i] = buffer.getLED(i).toHexString();
            }
            Logger.recordOutput("Leds/list", leds);
        }
        Logger.recordOutput("Leds/turret", turretState);
        Logger.recordOutput("Leds/side", sideState);
    }
}