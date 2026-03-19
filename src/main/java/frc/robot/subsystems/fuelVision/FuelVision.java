package frc.robot.subsystems.fuelVision;

import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.Collector;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import frc.utils.ExtraMath;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.FuelVisionConstants.*;

public class FuelVision extends SubsystemBase {
    private FuelVisionIO io;
    private FuelVisionIOInputsAutoLogged inputs = new FuelVisionIOInputsAutoLogged();
    private Supplier<Pose2d> pose;

    private ArrayList<fuelData> newFuel = new ArrayList<>();

    private Map<gridCoord, Set<fuelData>> fuelMap = new HashMap<>();

    public FuelVision(FuelVisionIO io, Supplier<Pose2d> pose){
        this.io = io;
        this.pose = pose;
    }


    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("IO/FuelVision", inputs);

//        newFuel.clear();

        for(FuelObservation o : inputs.observations){
            double d = (CAMERA_CONFIG.robotToCam.getZ()-FUEL_RADIUS.in(Meters))/
                    Math.tan(CAMERA_CONFIG.robotToCam.getRotation().getY()-o.screenPos().y())
                    /Math.cos(o.screenPos().x());

            if(d > MAX_DETECTION_DIST.in(Meters)){
                continue;
            }

            newFuel.add(new fuelData(toFieldReletive(new Translation2d(
                    d*Math.cos(-CAMERA_CONFIG.robotToCam.getRotation().getX()-o.screenPos().x()),
                    d*Math.sin(-CAMERA_CONFIG.robotToCam.getRotation().getX()-o.screenPos().x()))),
                    Logger.getTimestamp()));
        }

        for(Set<fuelData> dataSet : fuelMap.values()){
            for(fuelData data : dataSet){
                if(Microseconds.of(Logger.getTimestamp() - data.timestamp).gte(FUEL_PERSISTANCE_TIME)){
                    dataSet.remove(data);
                }
                //remove if inside fov
                if(ExtraMath.getAngleToPos(data.pos(), pose.get().getTranslation())
                        .minus(Radians.of(pose.get().getRotation().getRadians()))
                        .abs(Degrees) < CAMERA_HFOV.in(Degrees)/2){

                    dataSet.remove(data);
                }
            }
        }
        for(int i = 0; i < newFuel.size(); i++){
            fuelData d = newFuel.get(i);
            if(d.pos.getDistance(pose.get().getTranslation()) < 0.5){
                newFuel.remove(d);
                i--;
            } else {
                addToMap(d);
            }
        }

        Logger.recordOutput("Subsystems/Fuel Vision/current detected fuel", newFuel.stream().map(e -> new Translation3d(e.pos.getX(), e.pos.getY(), FUEL_RADIUS.in(Meters))).toArray(Translation3d[]::new));
        Logger.recordOutput("Subsystems/Fuel Vision/all mapped fuel", fuelMap.values().stream()
                .flatMap(Collection::stream)
                .map(e -> new Translation3d(e.pos.getX(), e.pos.getY(), FUEL_RADIUS.in(Meters)))
                .toArray(Translation3d[]::new));


        //trajectory to render fov
        Logger.recordOutput("Subsystems/Fuel Vision/fov", new Translation2d[]{
                pose.get().getTranslation().plus(CAMERA_CONFIG.robotToCam.getTranslation().toTranslation2d()),//camera pos

                new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)//one edge of fov
                        .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians)/2)))
                        .plus(pose.get().getTranslation()),

                new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                        .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians)/2.5)))
                        .plus(pose.get().getTranslation()),
                new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                        .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians)/3)))
                        .plus(pose.get().getTranslation()),
                new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                        .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians)/3.5)))
                        .plus(pose.get().getTranslation()),
                new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                        .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians)/4)))
                        .plus(pose.get().getTranslation()),
                new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                        .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians)/5)))
                        .plus(pose.get().getTranslation()),
                new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                        .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_HFOV.in(Radians)/6)))
                        .plus(pose.get().getTranslation()),
                //center
                new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                        .rotateBy(pose.get().getRotation())
                        .plus(pose.get().getTranslation()),


                new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                        .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians)/6)))
                        .plus(pose.get().getTranslation()),
                new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                        .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians)/5)))
                        .plus(pose.get().getTranslation()),
                new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                        .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians)/4)))
                        .plus(pose.get().getTranslation()),
                new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                        .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians)/3.5)))
                        .plus(pose.get().getTranslation()),
                new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                        .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians)/3)))
                        .plus(pose.get().getTranslation()),
                new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)
                        .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians)/2.5)))
                        .plus(pose.get().getTranslation()),

                new Translation2d(MAX_DETECTION_DIST.in(Meters), 0)//other edge of fov
                        .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_HFOV.in(Radians)/2)))
                        .plus(pose.get().getTranslation()),

                pose.get().getTranslation().plus(CAMERA_CONFIG.robotToCam.getTranslation().toTranslation2d()),//camera pos
        });
    }

    private void addToMap(fuelData data){
        int coordx = (int) (Math.floor(data.pos().getX()/GRID_SIZE.in(Meters)) * GRID_SIZE.in(Meters));
        int coordy = (int) (Math.floor(data.pos().getY()/GRID_SIZE.in(Meters)) * GRID_SIZE.in(Meters));

        fuelMap.getOrDefault(new gridCoord(coordx, coordy), new HashSet<>()).add(data);
    }

    private Translation2d toFieldReletive(Translation2d pos){
        return pos
                .rotateBy(new Rotation2d(pose.get().getRotation().getRadians()))
                .plus(new Translation2d(pose.get().getTranslation().getX(), pose.get().getTranslation().getY()));
    }

    public ArrayList<Translation2d> getNewFuel(){
        // yarden wrote this shit
        return newFuel.stream().map(e -> e.pos).collect(Collectors.toCollection(ArrayList::new));
    }

    public record gridCoord(int x, int y){}

    public record fuelData(Translation2d pos, long timestamp){}
}
