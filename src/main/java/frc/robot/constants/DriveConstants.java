// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import frc.utils.controlWrappers.PIDGains;

public class DriveConstants {
    // prevents drift when translating and rotation at the same time
    public static final double ANGULAR_VELOCITY_COEFFICIENT = 0.02;

    // rotation lock PIDs
    public static final PIDGains.PID ANGLE_PID = new PIDGains.PID(6, 0.0, 0.4);
    public static final PIDGains.PID ANGLE_PID_SIM = new PIDGains.PID(4, 0.0, 0.1);
    public static final AngularVelocity ANGLE_MAX_VELOCITY = RadiansPerSecond.of(11.2);

    // auto align tolerance
    public static final Angle AUTO_ALIGN_ANGLE_MAX_OFFSET = Degrees.of(0.05);
    public static final Distance AUTO_ALIGN_POS_MAX_OFFSET = Meters.of(0.01);

    // vision odometry enabled
    public static boolean USE_VISION = true;

    // pathplanner limits
    public static final LinearVelocity MAX_SPEED_PP = FeetPerSecond.of(15.5);
    public static final LinearAcceleration MAX_ACCEL_PP = MetersPerSecondPerSecond.of(2);
    public static final AngularVelocity MAX_ANGLE_SPEED_PP = RadiansPerSecond.of(10);
    public static final AngularAcceleration MAX_ANGLE_ACCEL_PP = RadiansPerSecondPerSecond.of(MAX_ANGLE_SPEED_PP.in(RadiansPerSecond) * 3);

    // pathplanner PIDs
    public static final PIDGains.PID AUTO_ANGLE_PID = new PIDGains.PID(8, 0.0, 0.0);
    public static final PIDGains.PID AUTO_ANGLE_PID_SIM = new PIDGains.PID(8, 0.5, 0.2);
    public static final PIDGains.PID TRANS_PID = new PIDGains.PID(5, 0.0, 0.0);
    public static final PIDGains.PID TRANS_PID_SIM = new PIDGains.PID(7, 1, 0.2);

    // kinematics
    // public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(4.7);
    public static final Frequency ODOMETRY_FREQ = Hertz.of(100.0);
    public static final Distance WIDTH = Inches.of(22); // size between wheels
    public static final Distance LENGTH = Inches.of(25);// size between wheels
    public static final Distance RADIUS = Meters.of(Math.hypot(WIDTH.in(Meters) / 2.0, LENGTH.in(Meters) / 2.0));
    public static final Translation2d[] MODULE_POSITIONS = new Translation2d[] {
            new Translation2d(WIDTH.in(Meters) / 2.0, LENGTH.in(Meters) / 2.0),
            new Translation2d(WIDTH.in(Meters) / 2.0, -LENGTH.in(Meters) / 2.0),
            new Translation2d(-WIDTH.in(Meters) / 2.0, LENGTH.in(Meters) / 2.0),
            new Translation2d(-WIDTH.in(Meters) / 2.0, -LENGTH.in(Meters) / 2.0)
    };

    // PathPlanner configuration
    public static final Mass MASS = Kilograms.of(60);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(6);
    public static final double COF = 2.31421199;
    public static final RobotConfig PP_CONFIG = new RobotConfig(
            MASS,
            MOI,
            new ModuleConfig(
                    module.WHEEL_RAD.in(Meters),
                    MAX_SPEED_PP.in(MetersPerSecond),
                    COF,
                    module.DRIVE_GEARBOX.withReduction(module.DRIVE_REDUCTION),
                    module.DRIVE_MAX_CURRENT.in(Amps),
                    1),
            MODULE_POSITIONS);

    // sysid
    public static final Voltage DRIVE_SYSID_VSTEP = Volts.of(7);
    public static final Velocity<VoltageUnit> DRIVE_SYSID_VRAMP = Volts.of(2).per(Second);
    public static final Time DRIVE_SYSID_TIMEOUT = Seconds.of(5);

    public static final Voltage TURN_SYSID_VSTEP = Volts.of(7);
    public static final Velocity<VoltageUnit> TURN_SYSID_VRAMP = Volts.of(1).per(Second);
    public static final Time TURN_SYSID_TIMEOUT = Seconds.of(10);

    // Device CAN IDs
    public static final int GYRO_ID = 30;

    public static final class module {

        public static final int FL_DRIVE_ID = 11;
        public static final int FR_DRIVE_ID = 12;
        public static final int BL_DRIVE_ID = 13;
        public static final int BR_DRIVE_ID = 14;

        public static final int FL_TURN_ID = 21;
        public static final int FR_TURN_ID = 22;
        public static final int BL_TURN_ID = 23;
        public static final int BR_TURN_ID = 24;

        // Drive motor configuration
        public static final boolean DRIVE_INVERT = true;
        public static final Current DRIVE_MAX_CURRENT = Amps.of(30);
        public static final double DRIVE_REDUCTION = 6.75;
        public static final Distance WHEEL_RAD = Inches.of(2);
        public static final DCMotor DRIVE_GEARBOX = DCMotor.getNEO(1);

        // Drive encoder configuration
        // NOTE: CTRE TalonFX takes gear reduciton into account when using getPosition()
        public static final double DRIVE_ENCODER_POS_FACTOR = 2 * Math.PI; // Mech Rotations -> Wheel Radians
        // NOTE: CTRE TalonFX returns Mechanism Rotations per second not rpm like the Spark MAX, it also takes configured gear reductions into account when running getVelocity() instead of getRotorVelocity()
        public static final double DRIVE_ENCODER_VEL_FACTOR = (2 * Math.PI); // Mech RPS -> Wheel RAD/Sec

        // Drive PID configuration
        public static final PIDGains.PID DRIVE_PID = new PIDGains.PID(0.0, 0.0, 0.0).makeTunable("Drive PID");
        public static final PIDGains.SimpleFF DRIVE_FF = new PIDGains.SimpleFF(0.15, 0.1, 0.0).makeTunable("Drive FF");// TODO sysid 0.11, 0.13, 0.1
        public static final PIDGains.PID DRIVE_PID_SIM = new PIDGains.PID(0.01, 0.0, 0.0);
        public static final PIDGains.SimpleFF DRIVE_FF_SIM = new PIDGains.SimpleFF(0.11, 0.13, 0.1);

        // Turn motor configuration
        public static final boolean TURN_INVERT = true;
        public static final Current TURN_CURRENT_LIM = Amps.of(10);
        public static final double TURN_REDUCTION = 21.428;
        public static final DCMotor TURN_GEARBOX = DCMotor.getNEO(1);

        // Turn encoder configuration
        public static final boolean TURN_ENCODER_INVERT = true;
        public static final double TURN_ENCODER_POS_FACTOR = 2 * Math.PI; // Rotations -> Radians
        public static final double TURN_ENCODER_VEL_FACTOR = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

        // Turn PID configuration
        public static final PIDGains.ProfiledPID TURN_PID = new PIDGains.ProfiledPID(7.5, 0.0, 0.0, Math.PI * 8,
                Math.PI * 80);
        public static final PIDGains.SimpleFF TURN_FF = new PIDGains.SimpleFF(0.125, 0.0, 0.0);
        public static final PIDGains.ProfiledPID TURN_PID_SIM = new PIDGains.ProfiledPID(12.5, 0.0, 0.5, Math.PI * 8,
                Math.PI * 80);
        public static final PIDGains.SimpleFF TURN_FF_SIM = new PIDGains.SimpleFF(0.015, 0.0, 0.0);

        public static final Angle TURN_MIN_POS = Radians.of(0);
        public static final Angle TURN_MAX_POS = Radians.of(2 * Math.PI);

        // factor to offset drive velocity based on turn velocity to reduce shift while turning
        public static final double DRIVE_OFFSET_VEL_FACTOR = 0.0;
    }
}