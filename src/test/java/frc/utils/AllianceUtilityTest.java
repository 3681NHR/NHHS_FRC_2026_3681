package frc.utils;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.AfterEach;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

import java.lang.reflect.Field;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceUtilityTest {

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
    public void flipPose_blue_returnsOriginal() throws Exception {
        setAlliance(Alliance.Blue);
        Pose2d original = new Pose2d(2, 3, Rotation2d.fromDegrees(45));
        Pose2d flipped = AllianceUtility.flipPose(original);
        assertEquals(original.getX(), flipped.getX(), 1e-9);
        assertEquals(original.getY(), flipped.getY(), 1e-9);
        assertEquals(original.getRotation().getDegrees(), flipped.getRotation().getDegrees(), 1e-9);
    }

    @Test
    public void flipPose_red_mirrorsAroundCenter() throws Exception {
        setAlliance(Alliance.Red);
        // field center at (8.27, 4.035)
        Pose2d original = new Pose2d(2, 3, Rotation2d.fromDegrees(30));
        Pose2d flipped = AllianceUtility.flipPose(original);
        // x' = center*2 - x = 16.54 - 2 = 14.54
        assertEquals(16.54 - 2, flipped.getX(), 1e-9);
        assertEquals(8.07 - 3, flipped.getY(), 1e-9);
        // 30+180=210 normalizes to -150 in Rotation2d range [-180,180)
        assertEquals(-150, flipped.getRotation().getDegrees(), 1e-6);
    }

    @Test
    public void flipPose_red_center_staysCenter() throws Exception {
        setAlliance(Alliance.Red);
        Pose2d center = AllianceUtility.FIELD_CENTER_POINT;
        Pose2d flipped = AllianceUtility.flipPose(center);
        assertEquals(center.getX(), flipped.getX(), 1e-9);
        assertEquals(center.getY(), flipped.getY(), 1e-9);
    }

    @Test
    public void flipPose_translation3d() throws Exception {
        setAlliance(Alliance.Red);
        Translation3d t = new Translation3d(2, 3, 1);
        Translation3d flipped = AllianceUtility.flipPose(t);
        assertEquals(14.54, flipped.getX(), 1e-9);
        assertEquals(5.07, flipped.getY(), 1e-9);
        assertEquals(1, flipped.getZ(), 1e-9);
    }

    @Test
    public void flipRectZone_blue_identity() throws Exception {
        setAlliance(Alliance.Blue);
        RectZone z = new RectZone(0, 0, 4, 8.07);
        RectZone flipped = AllianceUtility.flipRectZone(z);
        assertEquals(z.minX, flipped.minX, 1e-9);
        assertEquals(z.maxX, flipped.maxX, 1e-9);
    }

    @Test
    public void flipRectZone_red_mirrors() throws Exception {
        setAlliance(Alliance.Red);
        RectZone z = new RectZone(0, 0, 4, 8.07);
        RectZone flipped = AllianceUtility.flipRectZone(z);
        // minX 0 -> 16.54, maxX 4 -> 12.54, after normalization min=12.54, max=16.54
        assertEquals(12.54, flipped.minX, 1e-9);
        assertEquals(16.54, flipped.maxX, 1e-9);
    }

    @Test
    public void forceFlipRectZone_alwaysFlips() throws Exception {
        setAlliance(Alliance.Blue);
        RectZone z = new RectZone(0, 0, 4, 8.07);
        RectZone forced = AllianceUtility.forceFlipRectZone(z);
        assertEquals(12.54, forced.minX, 1e-9);
        assertEquals(16.54, forced.maxX, 1e-9);
    }
}
