package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(71+47);
    private AddressableLEDBufferView sideBuffer = new AddressableLEDBufferView(buffer, 71, 71+46);
    private AddressableLEDBufferView turretBuffer = new AddressableLEDBufferView(buffer, 0, 71);

    private final int turretBufferLength = turretBuffer.getLength();
    private static final int LEDTurretZeroOffset = 30;

    private LoggedNetworkBoolean rainbow = new LoggedNetworkBoolean("Debug/LED override", false);
    private LoggedNetworkBoolean coolTurretTrackLedThing = new LoggedNetworkBoolean("Overrides/Cool Turret Track Led Thing", true);
    private LoggedNetworkBoolean coolTurretTrackLedThingButItAlsoHelpsWithTurretDebug = new LoggedNetworkBoolean("Overrides/Cool Turret Track Led Thing But It Also Helps With Turret Debug", false);


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

        Color turretState = turret.isReady() && launcher.isReady() && hood.isReady() ? Color.kGreen : Color.kBlack;
        if(manualSupplier.getAsBoolean()){
            turretState = Color.kWhite;
        }
        if(!hood.isHomed()){
            turretState = Color.kRed;
        }
        if(hood.isHoming()){
            turretState = Color.kOrange;
        }

        int turretAbsolutePosIndexLed = (int) (ExtraMath.wrap(turret.getAbsoluteAngle().in(Rotations), 1) * turretBufferLength);
        turretAbsolutePosIndexLed += LEDTurretZeroOffset;
        turretAbsolutePosIndexLed %= turretBufferLength;
        int turretPosIndexLed = (int) (ExtraMath.wrap(turret.getAngle().in(Rotations), 1) * turretBufferLength);
        turretPosIndexLed += LEDTurretZeroOffset;
        turretPosIndexLed %= turretBufferLength;
        
        if(rainbow.get()){
            LEDPattern.rainbow(255, 255).applyTo(turretBuffer);
            LEDPattern.rainbow(255, 255).applyTo(sideBuffer);
        } else {
            LEDPattern.solid(turretState).atBrightness(Percent.of(50)).applyTo(turretBuffer);
            
            if (coolTurretTrackLedThing.get() && !coolTurretTrackLedThingButItAlsoHelpsWithTurretDebug.get()) {
                turretBuffer.setLED(turretAbsolutePosIndexLed, Color.kHotPink);
                for (int i = 1; i < 5; i++) {
                    int ledIndexPlus = (int) ExtraMath.wrap(turretAbsolutePosIndexLed + i, turretBufferLength);
                    int ledIndexMinus = (int) ExtraMath.wrap(turretAbsolutePosIndexLed - i, turretBufferLength);
                    if (ledIndexMinus < 0) {
                        ledIndexMinus += turretBufferLength;
                    }

                    turretBuffer.setLED(ledIndexPlus, Color.kHotPink);
                    turretBuffer.setLED(ledIndexMinus, Color.kHotPink);
                }
            }

            if (coolTurretTrackLedThingButItAlsoHelpsWithTurretDebug.get()) {
                turretBuffer.setLED(turretPosIndexLed, Color.kRed);
                turretBuffer.setLED(turretAbsolutePosIndexLed, Color.kGreen);
            }

            LEDPattern.solid(drive.getFOD() ? new Color() : Color.kBlue).atBrightness(Percent.of(50)).applyTo(sideBuffer);
        }

        led.setData(buffer);

        String[] leds = new String[buffer.getLength()];
        for (int i = 0; i < buffer.getLength(); i++) {
            leds[i] = buffer.getLED(i).toHexString();
        }
        Logger.recordOutput("Leds/list", leds);
    }
}