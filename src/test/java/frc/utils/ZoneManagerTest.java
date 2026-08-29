package frc.utils;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.AfterEach;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.lang.reflect.Field;

public class ZoneManagerTest {

    private void setAlliance(Alliance alliance) throws Exception {
        Field f = AllianceUtility.class.getDeclaredField("latestAlliance");
        f.setAccessible(true);
        f.set(null, alliance);
    }

    @AfterEach
    public void resetAlliance() throws Exception {
        setAlliance(Alliance.Blue);
    }

    @Test
    public void hubZone_blue_inside() throws Exception {
        setAlliance(Alliance.Blue);
        // hub = (-0.5, -0.5) to (4, 8.07) -> point (2,4) is inside
        ZoneManager.updateRobotPose(new Pose2d(2, 4, Rotation2d.kZero));
        assertEquals(ZoneManager.FieldZone.HUB, ZoneManager.getZone());
    }

    @Test
    public void hubZone_red_flipped() throws Exception {
        setAlliance(Alliance.Red);
        // on red, hub flips to (12.54,0) to (17.04, 8.57) approx -> point near red hub
        ZoneManager.updateRobotPose(new Pose2d(14, 4, Rotation2d.kZero));
        assertEquals(ZoneManager.FieldZone.HUB, ZoneManager.getZone());
    }

    @Test
    public void trenchOverridesHub() throws Exception {
        setAlliance(Alliance.Blue);
        // left trench 3.8,6.87 to 5.3,8.07
        ZoneManager.updateRobotPose(new Pose2d(4.5, 7.5, Rotation2d.kZero));
        assertEquals(ZoneManager.FieldZone.TRENCH, ZoneManager.getZone());
    }

    @Test
    public void passingZone() throws Exception {
        setAlliance(Alliance.Blue);
        // passing 5.2,0 to 16.5,8.07 but not in trench
        ZoneManager.updateRobotPose(new Pose2d(10, 4, Rotation2d.kZero));
        assertEquals(ZoneManager.FieldZone.PASS, ZoneManager.getZone());
    }

    @Test
    public void unknownZone() throws Exception {
        setAlliance(Alliance.Blue);
        // just outside hub but not in passing? Actually passing starts at 5.2, so 4.5,4 with trench y not in trench -> between hub and passing = unknown
        ZoneManager.updateRobotPose(new Pose2d(4.5, 4, Rotation2d.kZero));
        assertEquals(ZoneManager.FieldZone.UNKNOWN, ZoneManager.getZone());
    }

    @Test
    public void rightTrench() throws Exception {
        setAlliance(Alliance.Blue);
        ZoneManager.updateRobotPose(new Pose2d(4.5, 0.5, Rotation2d.kZero));
        assertEquals(ZoneManager.FieldZone.TRENCH, ZoneManager.getZone());
    }
}
