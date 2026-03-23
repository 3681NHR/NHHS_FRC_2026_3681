package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;

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

import frc.utils.ExtraMath;

public class Led extends SubsystemBase {

    Launcher launcher;
    Hood hood;
    Turret turret;
    Drive drive;

    BooleanSupplier manualSupplier;

    private final AddressableLED led = new AddressableLED(1);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(70+5);
    private final AddressableLEDBufferView sideBuffer = new AddressableLEDBufferView(buffer, 70, 70+4);
    private final AddressableLEDBufferView turretBuffer = new AddressableLEDBufferView(buffer, 0, 70);

    private final int turretBufferLength = turretBuffer.getLength();
    private final int LEDTurretZeroOffset = turretBufferLength-16;

    private LoggedNetworkBoolean coolTurretTrackLedThing = new LoggedNetworkBoolean("Overrides/Cool Turret Track Led Thing", true);
    private LoggedNetworkBoolean coolTurretTrackLedThingButItAlsoHelpsWithTurretDebug = new LoggedNetworkBoolean("Overrides/Cool Turret Track Led Thing But It Also Helps With Turret Debug", false);

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

        int turretAbsolutePosIndexLed = (int) (ExtraMath.wrap(turret.getAbsoluteAngle().times(-1).in(Rotations), 1) * turretBufferLength);
        turretAbsolutePosIndexLed += LEDTurretZeroOffset;
        turretAbsolutePosIndexLed %= turretBufferLength;
        int turretPosIndexLed = (int) (ExtraMath.wrap(turret.getAngle().times(-1).in(Rotations), 1) * turretBufferLength);
        turretPosIndexLed += LEDTurretZeroOffset;
        turretPosIndexLed %= turretBufferLength;
        Color sideState = drive.getFOD() ? new Color() : Color.kBlue;
        
        
        if(rainbow.get()){
            LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Hertz.of(0.5)).applyTo(turretBuffer);
            LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Hertz.of(0.5)).applyTo(sideBuffer);
        } else {
            LEDPattern.solid(turretState).atBrightness(Percent.of(50)).applyTo(turretBuffer);
            
            if (coolTurretTrackLedThing.get() && !coolTurretTrackLedThingButItAlsoHelpsWithTurretDebug.get()) {
                turretBuffer.setLED(turretPosIndexLed, Color.kMagenta);
                for (int i = 1; i < 5; i++) {
                    int ledIndexPlus = (int) ExtraMath.wrap(turretPosIndexLed + i, turretBufferLength);
                    int ledIndexMinus = (int) ExtraMath.wrap(turretPosIndexLed - i, turretBufferLength);
                    if (ledIndexMinus < 0) {
                        ledIndexMinus += turretBufferLength;
                    }

                    turretBuffer.setLED(ledIndexPlus, Color.kMagenta);
                    turretBuffer.setLED(ledIndexMinus, Color.kMagenta);
                }
            }

            if (coolTurretTrackLedThingButItAlsoHelpsWithTurretDebug.get()) {
                turretBuffer.setLED(turretPosIndexLed, Color.kRed);
                turretBuffer.setLED(turretAbsolutePosIndexLed, Color.kGreen);
            }

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