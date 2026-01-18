package frc.utils;

import java.util.HashMap;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.traits.SupportsMusic;

/**
 * singleton for playing music on connected TalonFX devices
 */
public class Symphony {
    private final Orchestra orchestra = new Orchestra();

    private HashMap<Long, SupportsMusic> instruments = new HashMap<>();
    
    private static Symphony instance;

    private Symphony() {}

    /**
     * get the Symphony singleton
     * @return [Symphony] symphony instance
     */
    public static Symphony getSymphony() {
        if (instance == null) {
            instance = new Symphony();
        }
        return instance;
    }

    public void registerInstrument(SupportsMusic instrument) {
        if (!instruments.containsKey(instrument.getDeviceHash())) {
            orchestra.addInstrument(instrument);
            instruments.put(instrument.getDeviceHash(), instrument);
        }
    }

    /**
     * Loads a Chirp file at the specified file path.
     * @param filePath
     */
    public void loadSong(String filePath) {
        orchestra.loadMusic(filePath);
    }

    /**
     * Plays the loaded music file.
     */
    public void play() {
        orchestra.play();
    }

    /**
     * Pauses the loaded music file.
     */
    public void pause() {
        orchestra.pause();
    }

    /**
     * Stops the loaded music file.
     */
    public void stop() {
        orchestra.stop();
    }

    /**
     * Gets whether the current track is actively playing.
     * @return [boolean] isPlaying
     */
    public boolean isPlaying() {
        return orchestra.isPlaying();
    }

    /**
     * Gets the current timestamp of the music file.
     * @return [double] timestamp
     */
    public double getCurrentTime() {
        return orchestra.getCurrentTime();
    }
}
