package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {

    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(71+47);
    private AddressableLEDBufferView side = new AddressableLEDBufferView(buffer, 71, 71+46);
    private AddressableLEDBufferView turret = new AddressableLEDBufferView(buffer, 0, 71);

    private LoggedNetworkBoolean rainbow = new LoggedNetworkBoolean("Debug/LED override", false);

    public Led() {
        led.setLength(buffer.getLength());

        led.start();
        
    }

    @Override
    public void periodic() {

        LEDPattern.rainbow(255, 255).applyTo(buffer);
        LEDPattern.solid(Color.kOrange).applyTo(side);

        led.setData(buffer);

        String[] leds = new String[buffer.getLength()];
        for (int i = 0; i < buffer.getLength(); i++) {
            leds[i] = buffer.getLED(i).toHexString();
        }
        Logger.recordOutput("Leds/list", leds);
    }
}