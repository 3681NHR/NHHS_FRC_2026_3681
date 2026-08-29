package frc.utils

import com.ctre.phoenix6.StatusCode
import edu.wpi.first.wpilibj.Alert
import java.util.function.Supplier

object PhoenixUtil {
    @JvmField var phoenixStickyFault: Boolean = false

    private val configFailureAlert = Alert(
        "Phoenix device configuration failed (see DriverStation logs).", Alert.AlertType.kError
    )

    @JvmStatic
    fun tryUntilOk(maxAttempts: Int, command: Supplier<StatusCode>): StatusCode {
        var status = StatusCode.OK
        for (i in 0 until maxAttempts) {
            status = command.get()
            if (status == StatusCode.OK) {
                return status
            }
        }
        phoenixStickyFault = true
        configFailureAlert.set(true)
        System.err.println(
            "PhoenixUtil.tryUntilOk: configuration failed after $maxAttempts attempts: $status"
        )
        return status
    }
}
