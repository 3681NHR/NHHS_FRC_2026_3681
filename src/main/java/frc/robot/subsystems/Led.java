package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {

    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(50);
    private AddressableLEDBufferView side = new AddressableLEDBufferView(buffer, 0, 5);
    private AddressableLEDBufferView turret = new AddressableLEDBufferView(buffer, 5, 10);

    private LoggedNetworkBoolean rainbow = new LoggedNetworkBoolean("Debug/LED override", false);



    public Led() {
        led.setLength(buffer.getLength());

        led.start();
    }

    @Override
    public void periodic() {

        

        led.setData(buffer);

        String[] leds = new String[buffer.getLength()];
        for (int i = 0; i < buffer.getLength(); i++) {
            leds[i] = buffer.getLED(i).toHexString();
        }
        Logger.recordOutput("Leds/list", leds);
    }
}