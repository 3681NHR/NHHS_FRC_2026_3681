package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

    public enum RobotMode {
        REAL,
        SIM,
        REPLAY
    }

    public static RobotMode SIM_MODE = RobotMode.SIM;// set to REPLAY to replay log files
    public static RobotMode MODE = RobotBase.isReal() ? RobotMode.REAL : SIM_MODE;

    public static final double AUTO_TIME = 15;
    public static final double TELEOP_TIME = 135;// 2:15
    public static final double ENDGAME_TIME = 20;// time remaining in teleop when endgame starts

    // default robot pose
    public static final Pose2d STARTING_POSE = new Pose2d(new Translation2d(8.75, 4), Rotation2d.fromDegrees(0));

    public static final double ROBOT_MASS = Units.lbsToKilograms(100);
    public static final double LOOP_TIME = 0.13; // 20ms + 110ms sprk max velocity lag

    public static class OperatorConstants {
        // Joystick Deadbands, radial from center
        public static final double LEFT_DEADBAND = 0.1;
        public static final double RIGHT_DEADBAND = 0.15;

        // stick curvature
        public static final double TRANSLATION_CURVE = 1.5;
        public static final double ROTATION_CURVE = 1.5;

        // usb ports of controllers, remember to assign controllers to their respective
        // ports in driverstation
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static class drive {
        //if robot should have field oriented drive enabled on start
        public static final boolean STARTING_FOD = true;
    }
}
