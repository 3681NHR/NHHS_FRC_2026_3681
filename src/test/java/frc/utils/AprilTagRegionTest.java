package frc.utils;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

public class AprilTagRegionTest {

    @Test
    public void both_containsRedAndBlue() {
        int[] both = AprilTagRegion.kStation.both();
        assertEquals(4, both.length);
        // station red {1,2} blue {12,13}
        assertArrayEquals(new int[]{1,2,12,13}, both);
    }

    @Test
    public void and_combines() {
        AprilTagRegion combined = AprilTagRegion.kStation.and(AprilTagRegion.kProcessor);
        assertArrayEquals(new int[]{1,2,3}, combined.red());
        assertArrayEquals(new int[]{12,13,16}, combined.blue());
    }

    @Test
    public void empty_hasNoTags() {
        assertEquals(0, AprilTagRegion.kEmpty.red().length);
        assertEquals(0, AprilTagRegion.kEmpty.blue().length);
        assertEquals(0, AprilTagRegion.kEmpty.both().length);
    }

    @Test
    public void reef_hasCorrectCounts() {
        assertEquals(6, AprilTagRegion.kReef.red().length);
        assertEquals(6, AprilTagRegion.kReef.blue().length);
        assertEquals(12, AprilTagRegion.kReef.both().length);
    }
}
