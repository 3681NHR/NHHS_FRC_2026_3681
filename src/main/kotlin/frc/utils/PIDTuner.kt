package frc.utils

import frc.utils.controlWrappers.PIDGains.Gains
import java.util.ArrayList

object PIDTuner {
    @JvmField
    var tunableGains: ArrayList<Gains> = ArrayList()

    @JvmStatic
    fun updateTunables() {
        for (g in tunableGains) {
            g.update()
        }
    }
}
