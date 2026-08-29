package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.LaunchLUT.ShotParams;

public class LaunchLUTTest {

    private static final double EPS = 1e-6;

    private static ShotParams[] makeSimpleLUT() {
        return new ShotParams[] {
            new ShotParams(Meters.of(1.0), Degrees.of(10), RPM.of(1000), Seconds.of(0.5)),
            new ShotParams(Meters.of(2.0), Degrees.of(20), RPM.of(2000), Seconds.of(1.0)),
            new ShotParams(Meters.of(3.0), Degrees.of(30), RPM.of(3000), Seconds.of(1.5))
        };
    }

    @Test
    public void get_exactMatch_returnsEntry() {
        ShotParams[] lut = makeSimpleLUT();
        ShotParams out = LaunchLUT.get(Meters.of(2.0), false, lut);
        assertEquals(20.0, out.hoodAngle().in(Degrees), EPS);
        assertEquals(2000.0, out.speed().in(RPM), EPS);
        assertEquals(1.0, out.time().in(Seconds), EPS);
    }

    @Test
    public void get_belowMin_returnsFirst() {
        ShotParams[] lut = makeSimpleLUT();
        ShotParams out = LaunchLUT.get(Meters.of(0.5), false, lut);
        assertEquals(10.0, out.hoodAngle().in(Degrees), EPS);
        // lerp false and true both return first entry when below min (i==0 path)
        ShotParams outLerp = LaunchLUT.get(Meters.of(0.5), true, lut);
        assertEquals(10.0, outLerp.hoodAngle().in(Degrees), EPS);
    }

    @Test
    public void get_lerp_interpolatesMidpoint() {
        ShotParams[] lut = makeSimpleLUT();
        // halfway between 1m and 2m
        ShotParams out = LaunchLUT.get(Meters.of(1.5), true, lut);
        assertEquals(15.0, out.hoodAngle().in(Degrees), EPS);
        assertEquals(1500.0, out.speed().in(RPM), EPS);
        assertEquals(0.75, out.time().in(Seconds), EPS);
        // dist field is set to query dist
        assertEquals(1.5, out.dist().in(Meters), EPS);
    }

    @Test
    public void get_noLerp_returnsUpperBound() {
        ShotParams[] lut = makeSimpleLUT();
        ShotParams out = LaunchLUT.get(Meters.of(1.5), false, lut);
        // should return upper entry (2m) not interpolated
        assertEquals(20.0, out.hoodAngle().in(Degrees), EPS);
        assertEquals(2000.0, out.speed().in(RPM), EPS);
    }

    @Test
    public void get_aboveMax_noLerp_returnsLast() {
        ShotParams[] lut = makeSimpleLUT();
        ShotParams out = LaunchLUT.get(Meters.of(10.0), false, lut);
        assertEquals(30.0, out.hoodAngle().in(Degrees), EPS);
        assertEquals(3000.0, out.speed().in(RPM), EPS);
    }

    @Test
    public void get_aboveMax_lerp_extrapolates() {
        ShotParams[] lut = makeSimpleLUT();
        // slope between last two: hood 10 deg per meter, speed 1000 rpm per meter, time 0.5 per meter
        // 4m is 1m beyond 3m: expect 40 deg, 4000 rpm, 2.0 sec
        ShotParams out = LaunchLUT.get(Meters.of(4.0), true, lut);
        assertEquals(40.0, out.hoodAngle().in(Degrees), 1e-3);
        assertEquals(4000.0, out.speed().in(RPM), 1e-3);
        assertEquals(2.0, out.time().in(Seconds), 1e-3);
        assertEquals(4.0, out.dist().in(Meters), EPS);
    }

    @Test
    public void get_infiniteDist_returnsZero() {
        ShotParams[] lut = makeSimpleLUT();
        Distance inf = Meters.of(Double.POSITIVE_INFINITY);
        ShotParams out = LaunchLUT.get(inf, true, lut);
        // LUT.length>1 is true, so infinite dist extrapolates via lerp with factor=Infinity
        // Actual Java behavior produces infinite outputs; preserve it.
        assertTrue(Double.isInfinite(out.hoodAngle().in(Radians)) || Double.isInfinite(out.speed().in(RPM)));
        // non-wrapping single-element path would return zero, but multi-element LUT extrapolates
    }

    @Test
    public void get_singleEntry_returnsZeroDistEntry() {
        ShotParams[] lut = new ShotParams[] {
            new ShotParams(Meters.of(2.0), Degrees.of(25), RPM.of(2500), Seconds.of(1.0))
        };
        // with single entry, lut.length >1 is false and dist is finite => returns zero-initialized out
        ShotParams out = LaunchLUT.get(Meters.of(2.0), true, lut);
        assertEquals(0.0, out.hoodAngle().in(Degrees), EPS);
    }

    // --- getSlope ---
    @Test
    public void getSlope_midpoint() {
        ShotParams[] lut = makeSimpleLUT();
        // between 1m and 2m: run = 1-2 = -1, hood diff = 10-20 = -10 => -10/-1 =10 deg/meter -> in radians per meter
        ShotParams slope = LaunchLUT.getSlope(Meters.of(1.5), lut);
        assertEquals(Math.toRadians(10.0), slope.hoodAngle().in(Radians), 1e-9);
        assertEquals(1000.0, slope.speed().in(RPM), EPS);
        assertEquals(0.5, slope.time().in(Seconds), EPS);
    }

    @Test
    public void getSlope_aboveMax_usesLastTwo() {
        ShotParams[] lut = makeSimpleLUT();
        ShotParams slope = LaunchLUT.getSlope(Meters.of(10.0), lut);
        // last two entries diff same as above
        assertEquals(Math.toRadians(10.0), slope.hoodAngle().in(Radians), 1e-9);
        assertEquals(1000.0, slope.speed().in(RPM), EPS);
    }

    @Test
    public void getSlope_belowMin_usesFirstTwo() {
        ShotParams[] lut = makeSimpleLUT();
        ShotParams slope = LaunchLUT.getSlope(Meters.of(0.5), lut);
        assertEquals(Math.toRadians(10.0), slope.hoodAngle().in(Radians), 1e-9);
    }
}
