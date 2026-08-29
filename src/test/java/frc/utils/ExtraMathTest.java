package frc.utils;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

public class ExtraMathTest {

    private static final double EPS = 1e-9;

    // --- wrap ---
    @Test
    public void wrap_withinRange() {
        assertEquals(1.5, ExtraMath.wrap(1.5, 10), EPS);
        assertEquals(0.0, ExtraMath.wrap(0, 10), EPS);
        assertEquals(9.9, ExtraMath.wrap(9.9, 10), EPS);
    }

    @Test
    public void wrap_negativeInput() {
        assertEquals(9.0, ExtraMath.wrap(-1, 10), EPS);
        assertEquals(5.0, ExtraMath.wrap(-5, 10), EPS);
        assertEquals(0.0, ExtraMath.wrap(-10, 10), EPS);
    }

    @Test
    public void wrap_overflowInput() {
        assertEquals(2.0, ExtraMath.wrap(12, 10), EPS);
        assertEquals(0.0, ExtraMath.wrap(10, 10), EPS);
        assertEquals(0.0, ExtraMath.wrap(20, 10), EPS);
    }

    // --- lerp / remap ---
    @Test
    public void lerp_basic() {
        assertEquals(0, ExtraMath.lerp(0, 10, 0), EPS);
        assertEquals(5, ExtraMath.lerp(0, 10, 0.5), EPS);
        assertEquals(10, ExtraMath.lerp(0, 10, 1.0), EPS);
        assertEquals(-5, ExtraMath.lerp(-10, 0, 0.5), EPS);
    }

    @Test
    public void remap_basic() {
        assertEquals(50, ExtraMath.remap(5, 0, 10, 0, 100), EPS);
        assertEquals(0, ExtraMath.remap(0, 0, 10, 0, 100), EPS);
        assertEquals(100, ExtraMath.remap(10, 0, 10, 0, 100), EPS);
        assertEquals(0, ExtraMath.remap(5, 0, 10, -50, 50), EPS);
    }

    // --- clamp (WPILib Measure) ---
    @Test
    public void clamp_withinRange() {
        assertEquals(5.0, ExtraMath.clamp(Meters.of(5), Meters.of(0), Meters.of(10)).in(Meters), EPS);
    }

    @Test
    public void clamp_belowMin() {
        assertEquals(0.0, ExtraMath.clamp(Meters.of(-5), Meters.of(0), Meters.of(10)).in(Meters), EPS);
    }

    @Test
    public void clamp_aboveMax() {
        assertEquals(10.0, ExtraMath.clamp(Meters.of(15), Meters.of(0), Meters.of(10)).in(Meters), EPS);
    }

    @Test
    public void clamp_atBoundaries() {
        assertEquals(0.0, ExtraMath.clamp(Meters.of(0), Meters.of(0), Meters.of(10)).in(Meters), EPS);
        assertEquals(10.0, ExtraMath.clamp(Meters.of(10), Meters.of(0), Meters.of(10)).in(Meters), EPS);
    }

    // --- mean ---
    @Test
    public void mean_basic() {
        assertEquals(2.0, ExtraMath.mean(1, 2, 3), EPS);
        assertEquals(5.0, ExtraMath.mean(5), EPS);
        assertEquals(0.0, ExtraMath.mean(-1, 1), EPS);
    }

    // --- lesser / greater (smallest/largest absolute value) ---
    @Test
    public void lesser_smallestAbs() {
        assertEquals(1.0, ExtraMath.lesser(1, -2, 3), EPS);
        assertEquals(-1.0, ExtraMath.lesser(-1, 2, 3), EPS);
        assertEquals(-0.5, ExtraMath.lesser(5, -0.5, 10), EPS);
    }

    @Test
    public void greater_largestAbs() {
        assertEquals(3.0, ExtraMath.greater(1, -2, 3), EPS);
        assertEquals(-5.0, ExtraMath.greater(2, -5, 3), EPS);
        assertEquals(10.0, ExtraMath.greater(10, -9, 2), EPS);
    }

    // --- min / max (actual min/max, not abs) ---
    @Test
    public void min_actualMin() {
        assertEquals(-2.0, ExtraMath.min(1, -2, 3), EPS);
        assertEquals(-5.0, ExtraMath.min(-5, -1, 0), EPS);
    }

    @Test
    public void max_actualMax() {
        assertEquals(3.0, ExtraMath.max(1, -2, 3), EPS);
        // NOTE: current Java impl is buggy for all-non-positive inputs: v > greaterAbs starts at 0,
        // so max(-5,-1,0) returns -Infinity rather than 0. Preserve buggy behavior for migration.
        assertEquals(Double.NEGATIVE_INFINITY, ExtraMath.max(-5, -1, 0), EPS);
        // positive-only still works
        assertEquals(5.0, ExtraMath.max(1, 5, 3), EPS);
    }

    // --- roundToPoint ---
    @Test
    public void roundToPoint_basic() {
        assertEquals(1.23, ExtraMath.roundToPoint(1.2345, 2), EPS);
        assertEquals(1.2, ExtraMath.roundToPoint(1.2345, 1), EPS);
        assertEquals(1.0, ExtraMath.roundToPoint(1.9, 0), EPS);
    }

    // --- getAngle / getMagnitude ---
    @Test
    public void getAngle_basic() {
        assertEquals(0.0, ExtraMath.getAngle(1, 0).getRadians(), EPS);
        assertEquals(Math.PI / 2, ExtraMath.getAngle(0, 1).getRadians(), EPS);
        assertEquals(Math.PI, ExtraMath.getAngle(-1, 0).getRadians(), EPS);
    }

    @Test
    public void getMagnitude_basic() {
        assertEquals(5.0, ExtraMath.getMagnitude(3, 4), EPS);
        assertEquals(0.0, ExtraMath.getMagnitude(0, 0), EPS);
    }

    // --- getTip ---
    @Test
    public void getTip_zeroTilt() {
        double[] tip = ExtraMath.getTip(new Rotation3d(0, 0, 0));
        assertEquals(0.0, tip[1], EPS);
    }

    @Test
    public void getTip_nonZero() {
        // tilt around X axis only
        double angle = 0.3;
        double[] tip = ExtraMath.getTip(new Rotation3d(angle, 0, 0));
        assertEquals(Math.abs(angle), tip[1], 1e-9);
    }

    // --- getAngleToPos ---
    @Test
    public void getAngleToPos_basic() {
        Translation2d curr = new Translation2d(0, 0);
        Translation2d target = new Translation2d(1, 0);
        assertEquals(0.0, ExtraMath.getAngleToPos(target, curr).in(Radians), EPS);

        target = new Translation2d(0, 1);
        assertEquals(Math.PI / 2, ExtraMath.getAngleToPos(target, curr).in(Radians), EPS);
    }

    // --- getDistance / poseWithinTolerance / getNearestPose ---
    @Test
    public void getDistance_basic() {
        Pose2d a = new Pose2d(0, 0, Rotation2d.kZero);
        Pose2d b = new Pose2d(3, 4, Rotation2d.kZero);
        assertEquals(5.0, ExtraMath.getDistance(a, b), EPS);
    }

    @Test
    public void poseWithinTolerance_true() {
        Pose2d a = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d b = new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(2));
        assertTrue(ExtraMath.poseWithinTolerance(a, b, 0.1, Math.toRadians(5)));
    }

    @Test
    public void poseWithinTolerance_falseLinear() {
        Pose2d a = new Pose2d(0, 0, Rotation2d.kZero);
        Pose2d b = new Pose2d(1, 0, Rotation2d.kZero);
        assertFalse(ExtraMath.poseWithinTolerance(a, b, 0.5, Math.toRadians(5)));
    }

    @Test
    public void poseWithinTolerance_falseAngular() {
        Pose2d a = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d b = new Pose2d(0, 0, Rotation2d.fromDegrees(10));
        assertFalse(ExtraMath.poseWithinTolerance(a, b, 0.1, Math.toRadians(5)));
    }

    @Test
    public void getNearestPose_basic() {
        Pose2d current = new Pose2d(1, 0, Rotation2d.kZero);
        Pose2d[] poses = new Pose2d[] {
            new Pose2d(0, 0, Rotation2d.kZero),
            new Pose2d(5, 0, Rotation2d.kZero),
            new Pose2d(1.1, 0, Rotation2d.kZero)
        };
        Pose2d nearest = ExtraMath.getNearestPose(poses, current);
        assertEquals(1.1, nearest.getX(), EPS);
    }

    // --- isNearState ---
    @Test
    public void isNearState_true() {
        State expected = new State(1.0, 2.0);
        State actual = new State(1.001, 2.001);
        State tol = new State(0.01, 0.01);
        assertTrue(ExtraMath.isNearState(expected, actual, tol));
    }

    @Test
    public void isNearState_false() {
        State expected = new State(1.0, 2.0);
        State actual = new State(1.1, 2.0);
        State tol = new State(0.01, 0.01);
        assertFalse(ExtraMath.isNearState(expected, actual, tol));
    }

    // --- processInput ---
    @Test
    public void processInput_noMods() {
        assertEquals(0.5, ExtraMath.processInput(0.5, null, null, null), EPS);
        assertEquals(-0.5, ExtraMath.processInput(-0.5, null, null, null), EPS);
    }

    @Test
    public void processInput_multiplier() {
        assertEquals(1.0, ExtraMath.processInput(0.5, 2.0, null, null), EPS);
        assertEquals(-1.0, ExtraMath.processInput(0.5, -2.0, null, null), EPS);
    }

    @Test
    public void processInput_deadband() {
        // inside deadband -> 0
        assertEquals(0.0, ExtraMath.processInput(0.05, null, null, 0.1), EPS);
        // outside deadband -> scaled
        double out = ExtraMath.processInput(0.5, null, null, 0.1);
        assertTrue(out > 0 && out < 0.5);
    }

    @Test
    public void processInput_square() {
        // square exponent 2: sign * |val|^2
        assertEquals(0.25, ExtraMath.processInput(0.5, null, 2.0, null), EPS);
        assertEquals(-0.25, ExtraMath.processInput(-0.5, null, 2.0, null), EPS);
    }

    // --- Derivitive ---
    @Test
    public void derivitive_withInit() {
        ExtraMath.Derivitive d = new ExtraMath.Derivitive(0.0);
        // first calculate: (2 - 0)/1 = 2
        assertEquals(2.0, d.calculate(2.0, 1.0), EPS);
        // next: (5 - 2)/1 = 3
        assertEquals(3.0, d.calculate(5.0, 1.0), EPS);
    }

    @Test
    public void derivitive_withoutInit() {
        ExtraMath.Derivitive d = new ExtraMath.Derivitive();
        // first call should set oldValue and return 0? Let's verify behavior: init false -> sets oldValue=measurement, init true, then (m - old)/dt where old == m => 0
        assertEquals(0.0, d.calculate(10.0, 1.0), EPS);
        assertEquals(5.0, d.calculate(15.0, 1.0), EPS);
    }

    @Test
    public void derivitive_reset() {
        ExtraMath.Derivitive d = new ExtraMath.Derivitive(0);
        d.calculate(10, 1);
        d.reset(0);
        assertEquals(7.0, d.calculate(7, 1), EPS);
    }

    // --- MovingAverageFilter ---
    @Test
    public void movingAverage_basic() {
        ExtraMath.MovingAverageFilter f = new ExtraMath.MovingAverageFilter(3);
        assertEquals(1.0, f.calculate(1.0), EPS);
        assertEquals(1.5, f.calculate(2.0), EPS);
        assertEquals(2.0, f.calculate(3.0), EPS);
        // window now [1,2,3] avg 2, next [2,3,4] avg 3
        assertEquals(3.0, f.calculate(4.0), EPS);
    }

    @Test
    public void movingAverage_reset() {
        ExtraMath.MovingAverageFilter f = new ExtraMath.MovingAverageFilter(2);
        f.calculate(10);
        f.calculate(20);
        f.reset();
        assertEquals(5.0, f.calculate(5), EPS);
    }
}
