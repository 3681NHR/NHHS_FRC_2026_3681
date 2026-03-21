package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.FuelVisionConstants;
import frc.robot.subsystems.fuelVision.FuelVision;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Drive;
import frc.utils.ExtraMath;
import frc.utils.RectZone;
import org.littletonrobotics.junction.Logger;
import org.w3c.dom.css.Rect;

import java.util.Comparator;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class DriveToFuel extends Command {

    Drive drive;
    FuelVision fuelVision;

    Command driveCommand;

    Supplier<RectZone> zone;

    double angleToTarg = 0;
    double fieldVX = 0;
    double fieldVY = 0;

    public DriveToFuel(Drive drive, FuelVision fuelVision, Supplier<RectZone> allowedZone) {
        this.zone = allowedZone;
        this.drive = drive;
        this.fuelVision = fuelVision;

        driveCommand = drive.rotationLock(
                () -> angleToTarg,
                () -> fieldVX,
                () -> fieldVY
        );
        CommandScheduler.getInstance().registerComposedCommands(driveCommand);

        addRequirements(drive, fuelVision);
    }

    @Override
    public void initialize() {
        driveCommand.initialize();
    }

    @Override
    public void execute() {
        Map<FuelVision.gridCoord, Set<FuelVision.fuelData>> map = fuelVision.getFuelMap();

        var best = map.entrySet().stream()
                .filter(entry -> zone.get().contains(new Translation2d(entry.getKey().x()* FuelVisionConstants.GRID_SIZE.in(Meters), entry.getKey().y()*FuelVisionConstants.GRID_SIZE.in(Meters))))
                .max(Comparator.comparingDouble(entry -> {
            int fuel = entry.getValue().size();
            double dist = new Translation2d(entry.getKey().x()* FuelVisionConstants.GRID_SIZE.in(Meters), entry.getKey().y()*FuelVisionConstants.GRID_SIZE.in(Meters)).getDistance(drive.getPose().getTranslation());
            return fuel/Math.max(dist, 0.01);
        }));
        if(best.isPresent() && !best.get().getValue().isEmpty()) {
            angleToTarg = ExtraMath.getAngleToPos(new Translation2d(best.get().getKey().x()* FuelVisionConstants.GRID_SIZE.in(Meters), best.get().getKey().y()*FuelVisionConstants.GRID_SIZE.in(Meters)), drive.getPose().getTranslation()).in(Radians);

            fieldVX = Math.cos(drive.getRotation().getRadians()-angleToTarg) * Math.cos(angleToTarg);
            fieldVY = Math.cos(drive.getRotation().getRadians()-angleToTarg) * Math.sin(angleToTarg);

            Logger.recordOutput("Auto/AI/target", new Translation2d(best.get().getKey().x()* FuelVisionConstants.GRID_SIZE.in(Meters), best.get().getKey().y()*FuelVisionConstants.GRID_SIZE.in(Meters)));
        } else {
            Logger.recordOutput("Auto/AI/target", ((Translation2d) null));
            angleToTarg = drive.getRotation().plus(Rotation2d.fromDegrees(90*0.02)).getRadians();
            fieldVX = 0;
            fieldVY = 0;
        }
        driveCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        driveCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return driveCommand.isFinished();
    }
}