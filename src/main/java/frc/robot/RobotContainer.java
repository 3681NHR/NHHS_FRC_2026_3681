/* I wrote this robot code with furry paws on. Just thought I would mention that. -yarden*/

package frc.robot;

import frc.robot.autos.AutoChooser;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOCrackingSpark;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPhoton;
import frc.robot.subsystems.vision.CameraIOPhotonSim;
import frc.robot.subsystems.vision.Vision;
import frc.utils.rumble.*;
import frc.utils.VariableLimSLR;
import frc.utils.Joystick.duelJoystickAxis;
import frc.utils.TimerHandler;
import frc.utils.BatteryVoltageSim;
import frc.utils.ExtraMath;
import frc.utils.Joystick;
import frc.utils.Symphony;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import static frc.utils.ControllerMap.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.utils.Alert;
import frc.utils.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    private DriveTrainSimulationConfig driveTrainSimulationConfig;
    private SwerveDriveSimulation driveSim;

    private Drive drive;
    private Vision vision;

    private Led led = new Led();

    private final XboxController driverController = new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final XboxController operatorController = new XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    private LoggedNetworkBoolean resetOdometry = new LoggedNetworkBoolean("resetOdometry", false);
    private AutoChooser autoChooser;

    private RumbleHandler rumbler = new RumbleHandler(driverController);
    private RumbleHandler opRumbler = new RumbleHandler(operatorController);

    private PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);

    private VariableLimSLR lxLim = new VariableLimSLR(Double.POSITIVE_INFINITY);
    private VariableLimSLR lyLim = new VariableLimSLR(Double.POSITIVE_INFINITY);
    private VariableLimSLR rxLim = new VariableLimSLR(Double.POSITIVE_INFINITY);
    private VariableLimSLR ryLim = new VariableLimSLR(Double.POSITIVE_INFINITY);

    AprilTagFieldLayout e = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
    private final Alert operatorDisconnected = new Alert("Operator controller disconnected (port 1).",
            AlertType.kWarning);

    private Superstructure superstructure;

    private duelJoystickAxis driverSticks;

    private Symphony symphony = Symphony.getSymphony();

    public RobotContainer() {

        try {
            // load test field layout for camera offset calculation, do not use otherwise
            // e = new AprilTagFieldLayout(Filesystem.getDeployDirectory().getAbsolutePath()
            // + "/test_field.json");
        } catch (Exception ex) {
        }

        Logger.recordOutput("AScope/zeroPose", new Pose3d());

        // we use our own warnings for joysticks
        DriverStation.silenceJoystickConnectionWarning(true);

        if (RobotBase.isSimulation()) {
            // maplesim setup
            driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
                    .withGyro(COTS.ofPigeon2())
                    .withSwerveModule(COTS.ofMark4i(
                            DCMotor.getNEO(1),
                            DCMotor.getNEO(1),
                            COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                            2))
                    .withTrackLengthTrackWidth(Meters.of(DriveConstants.LENGTH), Meters.of(DriveConstants.WIDTH))
                    .withBumperSize(Inches.of(31), Inches.of(33));

            driveSim = new SwerveDriveSimulation(driveTrainSimulationConfig, Constants.STARTING_POSE);
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
        }

        // process driver controls(radial deadzone, curve, trigger slowdown, and
        // inversion)
        driverSticks = new duelJoystickAxis(
                () -> lxLim.calculate(ExtraMath.processInput(
                        Joystick.deadzone(Constants.OperatorConstants.LEFT_DEADBAND,
                                driverController.getRawAxis(LEFT_STICK_X), driverController.getRawAxis(LEFT_STICK_Y))
                                .getX(),
                        -1.0,
                        Constants.OperatorConstants.TRANSLATION_CURVE, 0.0)),
                () -> lyLim.calculate(ExtraMath.processInput(
                        Joystick.deadzone(Constants.OperatorConstants.LEFT_DEADBAND,
                                driverController.getRawAxis(LEFT_STICK_X), driverController.getRawAxis(LEFT_STICK_Y))
                                .getY(),
                        -1.0,
                        Constants.OperatorConstants.TRANSLATION_CURVE, 0.0)),
                () -> rxLim.calculate(ExtraMath.processInput(
                        Joystick.deadzone(Constants.OperatorConstants.RIGHT_DEADBAND,
                                driverController.getRawAxis(RIGHT_STICK_X), driverController.getRawAxis(RIGHT_STICK_Y))
                                .getX(),
                        -0.75,
                        Constants.OperatorConstants.ROTATION_CURVE, 0.0)),
                () -> ryLim.calculate(ExtraMath.processInput(
                        Joystick.deadzone(Constants.OperatorConstants.RIGHT_DEADBAND,
                                driverController.getRawAxis(RIGHT_STICK_X), driverController.getRawAxis(RIGHT_STICK_Y))
                                .getY(),
                        -1.0,
                        Constants.OperatorConstants.ROTATION_CURVE, 0.0)));

        switch (Constants.MODE) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                vision = new Vision(
                        e,
                        new CameraIOPhoton(e, VisionConstants.CAMERA_CONFIGS[0]));
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOCrackingSpark(0),
                        new ModuleIOCrackingSpark(1),
                        new ModuleIOCrackingSpark(2),
                        new ModuleIOCrackingSpark(3),
                        vision,
                        driverSticks,
                        led);
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                vision = new Vision(
                        e,
                        new CameraIOPhotonSim(e, VisionConstants.CAMERA_CONFIGS[0],
                                driveSim::getSimulatedDriveTrainPose));
                if (driveSim != null) {
                    drive = new Drive(
                            new GyroIOSim(driveSim.getGyroSimulation()) {
                            },
                            new ModuleIOSim(driveSim.getModules()[0]),
                            new ModuleIOSim(driveSim.getModules()[1]),
                            new ModuleIOSim(driveSim.getModules()[2]),
                            new ModuleIOSim(driveSim.getModules()[3]),
                            vision,
                            driverSticks,
                            led);
                }
                break;

            default:
                // Replayed robot, disable IO implementations for replay
                vision = new Vision(
                        e,
                        new CameraIO() {
                        },
                        new CameraIO() {
                        },
                        new CameraIO() {
                        });
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        vision,
                        driverSticks,
                        led);
                break;
        }

        superstructure = new Superstructure(
                drive,
                vision,
                led,
                lxLim,
                lyLim,
                rxLim,
                ryLim);

        // build pathplanner autos and put in dashboard
        // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        configureBindings();

        autoChooser = new AutoChooser(this);

        LoggedPowerDistribution.getInstance(pdp.getModule(), ModuleType.kRev);
    }

    private void configureBindings() {
        // reset odometry dashboard button
        resetOdometry.set(false);
        new Trigger(() -> resetOdometry.get()).onTrue(new InstantCommand(() -> {
            resetOdometry.set(false);
            drive.setPose(Constants.STARTING_POSE);
        }));

        // send haptic command when 25 seconds are left in teleops
        new Trigger(() -> TimerHandler.getTeleopRemaining() < 25.0).onTrue(new InstantCommand(() -> {
            rumbler.overrideQue(RumblePreset.TAP.load());
            opRumbler.overrideQue(RumblePreset.TAP.load());
        }));

        // ------------------------------------------------------------------------------
        // driver controls
        // ------------------------------------------------------------------------------

        // move wheels to X, makes robot hard to push
        new Trigger(() -> driverController.getRawButton(LOGO_RIGHT)).whileTrue(new InstantCommand(() -> {
            drive.stopWithX();
        }, drive).repeatedly());

        // reset gyro angle
        new Trigger(() -> driverController.getRawButton(LOGO_LEFT)).onTrue(new InstantCommand(() -> {
            drive.resetGyro(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? Math.PI : 0);
            rumbler.overrideQue(RumblePreset.TAP.load());
        }));

        // override auto drive
        new Trigger(() -> driverController.getPOV() == 0).onTrue(drive.TeleopDrive());

        // state override
        new Trigger(() -> driverController.getPOV() == 180).or(() -> operatorController.getRawButton(LOGO_RIGHT))
                .onTrue(new InstantCommand(() -> {
                    superstructure.setWantedState(WantedSuperState.DEFAULT_STATE);
                }));

        // ------------------------------------------------------------------------------
        // operator controls
        // ------------------------------------------------------------------------------

    }

    public void Periodic() {
        rumbler.update(0.02);
        driverDisconnected.set(!driverController.isConnected());
        operatorDisconnected.set(!operatorController.isConnected());
    
        autoChooser.update();
    }

    public void SimPeriodic() {

        SimulatedArena.getInstance().simulationPeriodic();

        Logger.recordOutput("simulatedVoltage", BatteryVoltageSim.getInstance().calculateVoltage());
        Logger.recordOutput("FieldSimulation/RobotPose", driveSim.getSimulatedDriveTrainPose());

        Logger.recordOutput("FieldSimulation/Algae",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput("FieldSimulation/Coral",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    }

    public Command getAutonomousCommand() {
        // Command auto = autoChooser.get();
        Command auto = autoChooser.getSelected();
        return auto;
    }

    public void resetDrivetrain(Pose2d pose) {
        driveSim.setSimulationWorldPose(pose);
    }

    public void enableTeleop() {
        CommandScheduler.getInstance().schedule(drive.TeleopDrive());
        // symphony.loadSong("music/the-trout.chrp");
        // symphony.play();
        drive.setCallback();
    }

    public void enableAuto() {
    }

    public Drive getDrive() {
        return drive;
    }
}
