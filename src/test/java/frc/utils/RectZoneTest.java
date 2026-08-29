package frc.utils;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

public class RectZoneTest {

    @Test
    public void contains_inside() {
        RectZone z = new RectZone(0, 0, 10, 10);
        assertTrue(z.contains(new Translation2d(5, 5)));
        assertTrue(z.contains(new Translation2d(0, 0)));
        assertTrue(z.contains(new Translation2d(10, 10)));
    }

    @Test
    public void contains_outside() {
        RectZone z = new RectZone(0, 0, 10, 10);
        assertFalse(z.contains(new Translation2d(-1, 5)));
        assertFalse(z.contains(new Translation2d(5, 11)));
        assertFalse(z.contains(new Translation2d(11, 11)));
    }

    @Test
    public void constructor_normalizes() {
        // reversed inputs should still create correct zone
        RectZone z = new RectZone(10, 10, 0, 0);
        assertEquals(0, z.minX, 1e-9);
        assertEquals(0, z.minY, 1e-9);
        assertEquals(10, z.maxX, 1e-9);
        assertEquals(10, z.maxY, 1e-9);
        assertTrue(z.contains(new Translation2d(5, 5)));
    }

    @Test
    public void contains_onEdge() {
        RectZone z = new RectZone(1, 2, 3, 4);
        assertTrue(z.contains(new Translation2d(1, 3)));
        assertTrue(z.contains(new Translation2d(3, 3)));
        assertTrue(z.contains(new Translation2d(2, 2)));
        assertTrue(z.contains(new Translation2d(2, 4)));
    }
}
