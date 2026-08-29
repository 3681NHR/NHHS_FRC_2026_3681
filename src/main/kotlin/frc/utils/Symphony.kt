package frc.utils

import com.ctre.phoenix6.Orchestra
import com.ctre.phoenix6.hardware.traits.SupportsMusic

class Symphony private constructor() {
    private val orchestra = Orchestra()
    private val instruments: MutableMap<Long, SupportsMusic> = HashMap()

    fun registerInstrument(instrument: SupportsMusic) {
        if (!instruments.containsKey(instrument.deviceHash)) {
            orchestra.addInstrument(instrument)
            instruments[instrument.deviceHash] = instrument
        }
    }

    fun loadSong(filePath: String) {
        orchestra.loadMusic(filePath)
    }

    fun play() {
        orchestra.play()
    }

    fun pause() {
        orchestra.pause()
    }

    fun stop() {
        orchestra.stop()
    }

    fun isPlaying(): Boolean = orchestra.isPlaying

    fun getCurrentTime(): Double = orchestra.currentTime

    companion object {
        private var instance: Symphony? = null

        @JvmStatic
        fun getSymphony(): Symphony {
            if (instance == null) {
                instance = Symphony()
            }
            return instance!!
        }
    }
}
