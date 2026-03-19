package frc.robot.subsystems.fuelVision;

import java.util.*;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.utils.ExtraMath;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.FuelVisionConstants.*;

public class FuelVision extends SubsystemBase {
    private final FuelVisionIO io;
    private final FuelVisionIOInputsAutoLogged inputs = new FuelVisionIOInputsAutoLogged();
    private final Supplier<Pose2d> pose;

    private final ArrayList<fuelData> newFuel = new ArrayList<>();
    private ArrayList<fuelData> trackedFuel = new ArrayList<>();

    private final Map<gridCoord, Set<fuelData>> fuelMap = new HashMap<>();

    public FuelVision(FuelVisionIO io, Supplier<Pose2d> pose){
        this.io = io;
        this.pose = pose;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("IO/FuelVision", inputs);

        newFuel.clear();
        long currTimestamp = Logger.getTimestamp();

        for(FuelObservation o : inputs.observations){
            double d = (CAMERA_CONFIG.robotToCam.getZ()-FUEL_RADIUS.in(Meters))/
                    Math.tan(CAMERA_CONFIG.robotToCam.getRotation().getY()-o.screenPos().y())
                    /Math.cos(o.screenPos().x());

            //ignore fuel out of bounds
            if(d > MAX_DETECTION_DIST.in(Meters) || d < MIN_DETECTION_DIST.in(Meters)){
                continue;
            }

            newFuel.add(new fuelData(toFieldRelative(new Translation2d(
                    d*Math.cos(-CAMERA_CONFIG.robotToCam.getRotation().getX()-o.screenPos().x()),
                    d*Math.sin(-CAMERA_CONFIG.robotToCam.getRotation().getX()-o.screenPos().x()))),
                    currTimestamp));
        }

        updateTrackedFuel(currTimestamp);
        pruneFuelMap(currTimestamp);

        Logger.recordOutput("Subsystems/Fuel Vision/current detected fuel", newFuel.stream().map(e -> new Translation3d(e.pos.getX(), e.pos.getY(), FUEL_RADIUS.in(Meters))).toArray(Translation3d[]::new));
        Logger.recordOutput("Subsystems/Fuel Vision/all mapped fuel", getKnownFuel().stream()
                .map(e -> new Translation3d(e.getX(), e.getY(), FUEL_RADIUS.in(Meters)))
                .toArray(Translation3d[]::new));


        //trajectory to render fov
        {
            Logger.recordOutput("Subsystems/Fuel Vision/fov", new Translation2d[]{

                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)//one edge of fov
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians) / 2)))
                            .plus(pose.get().getTranslation()),

                    new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)//one edge of fov
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians) / 2)))
                            .plus(pose.get().getTranslation()),

                    new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians) / 2.5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians) / 3)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians) / 3.5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians) / 4)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians) / 5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians) / 6)))
                            .plus(pose.get().getTranslation()),
                    //center
                    new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation())
                            .plus(pose.get().getTranslation()),


                    new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians) / 6)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians) / 5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians) / 4)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians) / 3.5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians) / 3)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians) / 2.5)))
                            .plus(pose.get().getTranslation()),

                    new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)//other edge of fov
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians) / 2)))
                            .plus(pose.get().getTranslation()),

                    //inner rad
                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)//other edge of fov
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians) / 2)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians) / 2.5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians) / 3)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians) / 3.5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians) / 4)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians) / 5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians) / 6)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation())
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians) / 6)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians) / 5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians) / 4)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians) / 3.5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians) / 3)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians) / 2.5)))
                            .plus(pose.get().getTranslation()),

                    new Translation2d(MIN_DETECTION_DIST.in(Meters), 0)//one edge of fov
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians) / 2)))
                            .plus(pose.get().getTranslation()),
            });
        }
    }

    private void updateTrackedFuel(long nowUs) {
        List<fuelData> unmatchedDetections = new ArrayList<>(newFuel);
        ArrayList<fuelData> nextTrackedFuel = new ArrayList<>();

        for (fuelData oldTrack : trackedFuel) {
            //deduplicate
            int bestMatchIndex = findBestMatch(oldTrack, unmatchedDetections);
            if (bestMatchIndex >= 0) {
                fuelData matched = unmatchedDetections.remove(bestMatchIndex);
                nextTrackedFuel.add(matched);
            } else if (!isInCameraPov(oldTrack.pos())) {
                addToMap(new fuelData(oldTrack.pos(), nowUs));
            }
        }

        nextTrackedFuel.addAll(unmatchedDetections);
        trackedFuel = nextTrackedFuel;

        for (fuelData visible : trackedFuel) {
            removeFromMap(visible.pos());
        }
    }

    private int findBestMatch(fuelData reference, List<fuelData> detections) {
        int bestIndex = -1;
        double bestDistance = Double.POSITIVE_INFINITY;
        for (int i = 0; i < detections.size(); i++) {
            double distance = reference.pos().getDistance(detections.get(i).pos());
            if (distance <= FUEL_OVERLAP_THRESH.in(Meters) && distance < bestDistance) {
                bestDistance = distance;
                bestIndex = i;
            }
        }
        return bestIndex;
    }

    private boolean isInCameraPov(Translation2d fieldPos) {
        Pose2d robotPose = pose.get();
        Translation2d cameraOffset = CAMERA_CONFIG.robotToCam.getTranslation().toTranslation2d().rotateBy(robotPose.getRotation());
        Translation2d cameraPos = robotPose.getTranslation().plus(cameraOffset);
        Rotation2d cameraYaw = robotPose.getRotation().plus(new Rotation2d(CAMERA_CONFIG.robotToCam.getRotation().getX()));

        double distance = cameraPos.getDistance(fieldPos);
        if (distance > MAX_DETECTION_DIST.in(Meters)) {
            return false;
        }

        double angleError = Math.abs(MathUtil.angleModulus(
                ExtraMath.getAngleToPos(fieldPos, cameraPos).in(Radians) - cameraYaw.getRadians()));
        return angleError <= CAMERA_HFOV.in(Radians) / 2.0;
    }

    private void pruneFuelMap(long nowUs) {
        Iterator<Map.Entry<gridCoord, Set<fuelData>>> cellIterator = fuelMap.entrySet().iterator();
        while (cellIterator.hasNext()) {
            Map.Entry<gridCoord, Set<fuelData>> cell = cellIterator.next();
            cell.getValue().removeIf(data -> {
                boolean stale = Microseconds.of(nowUs - data.timestamp).gte(FUEL_PERSISTANCE_TIME);
                boolean redetected = overlapsTrackedFuel(data.pos());
                // Debounce POV-only clearing so fast camera sweeps do not purge the cache.
                boolean inPovLongEnoughToClear = isInCameraPov(data.pos())
                        && Microseconds.of(nowUs - data.timestamp).gte(FUEL_POV_CLEAR_GRACE_TIME);
                return stale || redetected || inPovLongEnoughToClear;
            });
            if (cell.getValue().isEmpty()) {
                cellIterator.remove();
            }
        }
    }

    private boolean overlapsTrackedFuel(Translation2d pos) {
        for (fuelData tracked : trackedFuel) {
            if (tracked.pos().getDistance(pos) <= FUEL_OVERLAP_THRESH.in(Meters)) {
                return true;
            }
        }
        return false;
    }

    private void addToMap(fuelData data){
        int coordx = (int) (Math.floor(data.pos().getX()/GRID_SIZE.in(Meters)) * GRID_SIZE.in(Meters));
        int coordy = (int) (Math.floor(data.pos().getY()/GRID_SIZE.in(Meters)) * GRID_SIZE.in(Meters));
        gridCoord key = new gridCoord(coordx, coordy);
        Set<fuelData> mapCell = fuelMap.computeIfAbsent(key, k -> new HashSet<>());
        mapCell.removeIf(existing -> existing.pos().getDistance(data.pos()) <= FUEL_OVERLAP_THRESH.in(Meters));
        mapCell.add(data);
    }

    private void removeFromMap(Translation2d pos) {
        Iterator<Map.Entry<gridCoord, Set<fuelData>>> cellIterator = fuelMap.entrySet().iterator();
        while (cellIterator.hasNext()) {
            Map.Entry<gridCoord, Set<fuelData>> cell = cellIterator.next();
            cell.getValue().removeIf(data -> data.pos().getDistance(pos) <= FUEL_OVERLAP_THRESH.in(Meters));
            if (cell.getValue().isEmpty()) {
                cellIterator.remove();
            }
        }
    }

    private Translation2d toFieldRelative(Translation2d pos){
        Translation2d cameraOffset = CAMERA_CONFIG.robotToCam.getTranslation().toTranslation2d().rotateBy(pose.get().getRotation());

        return pos
                .rotateBy(new Rotation2d(pose.get().getRotation().getRadians()))
                .plus(new Translation2d(pose.get().getTranslation().getX(), pose.get().getTranslation().getY()))
                .plus(cameraOffset);
    }

    public ArrayList<Translation2d> getNewFuel(){
        return newFuel.stream().map(fuelData::pos).collect(Collectors.toCollection(ArrayList::new));
    }

    public ArrayList<Translation2d> getTrackedFuel(){
        return trackedFuel.stream().map(fuelData::pos).collect(Collectors.toCollection(ArrayList::new));
    }

    public ArrayList<Translation2d> getKnownFuel(){
        ArrayList<Translation2d> known = getTrackedFuel();
        fuelMap.values().stream()
                .flatMap(Collection::stream)
                .map(fuelData::pos)
                .filter(cached -> known.stream().noneMatch(current -> current.getDistance(cached) <= FUEL_OVERLAP_THRESH.in(Meters)))
                .forEach(known::add);
        return known;
    }

    public record gridCoord(int x, int y){}

    public record fuelData(Translation2d pos, long timestamp){}
}
