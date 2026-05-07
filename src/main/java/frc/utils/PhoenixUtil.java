package frc.utils;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Utility helpers for CTRE Phoenix devices. Mirrors {@link SparkUtil} but for
 * Phoenix's {@link StatusCode} based APIs. Configuration calls on TalonFX,
 * CANcoder, etc. return a {@link StatusCode}; ignoring it can cause silent
 * hardware misconfiguration (e.g. inversion or current limits not actually
 * applying), which is a safety regression for the robot.
 */
public class PhoenixUtil {
    /** Sticky flag set whenever a Phoenix config call fails after all retries. */
    public static boolean phoenixStickyFault = false;

    private static final Alert configFailureAlert = new Alert(
            "Phoenix device configuration failed (see DriverStation logs).", AlertType.kError);

    /**
     * Attempts to run the supplied Phoenix config command until it returns
     * {@link StatusCode#OK} or {@code maxAttempts} have been exhausted. Returns
     * the last {@link StatusCode}. On failure, sets {@link #phoenixStickyFault}
     * and raises a driver-station alert so the failure is surfaced instead of
     * being swallowed.
     */
    public static StatusCode tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        StatusCode status = StatusCode.OK;
        for (int i = 0; i < maxAttempts; i++) {
            status = command.get();
            if (status.equals(StatusCode.OK)) {
                return status;
            }
        }
        phoenixStickyFault = true;
        configFailureAlert.set(true);
        System.err.println("PhoenixUtil.tryUntilOk: configuration failed after "
                + maxAttempts + " attempts: " + status.toString());
        return status;
    }
}
