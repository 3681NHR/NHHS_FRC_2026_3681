package frc.robot.subsystems.fuelVision;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.constants.FuelVisionConstants.*;

public class FuelVision extends SubsystemBase {
    private FuelVisionIO io;
    private FuelVisionIOInputsAutoLogged inputs = new FuelVisionIOInputsAutoLogged();
    private Supplier<Pose2d> pose;

    public FuelVision(FuelVisionIO io, Supplier<Pose2d> pose){
        this.io = io;
        this.pose = pose;
    }


    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("IO/FuelVision", inputs);

        ArrayList<Translation3d> radialEstimates = new ArrayList<Translation3d>();
        ArrayList<Double> distEstimates = new ArrayList<Double>();
        ArrayList<Double> heightEstimates = new ArrayList<Double>();
        ArrayList<Double> hDistEstimates = new ArrayList<Double>();
        ArrayList<Double> thetaEstimates = new ArrayList<Double>();
        ArrayList<Double> areaEstimates = new ArrayList<Double>();

        for(FuelObservation o : inputs.observations){
            double areaEst = 100 * Math.pow(Math.max(o.screensize().x(), o.screensize().y()), 2);
            areaEstimates.add(areaEst);

            double dist = FUEL_SIZE_BASELINE/Math.sqrt(areaEst);
            distEstimates.add(dist);

            double theta = -CAMERA_CONFIG.robotToCam.getRotation().getY()+o.screenPos().y().in(Radians);
            thetaEstimates.add(theta);
            
            double height = Math.sin(theta) * dist;
            heightEstimates.add(height);

            double hDist = Math.cos(theta) * dist;
            hDistEstimates.add(hDist);

            radialEstimates.add(new Translation3d(
               Math.cos(CAMERA_CONFIG.robotToCam.getRotation().getZ() - o.screenPos().x().in(Radians)) * hDist,
               Math.sin(CAMERA_CONFIG.robotToCam.getRotation().getZ() - o.screenPos().x().in(Radians)) * hDist,
               height 
            )
            .plus(CAMERA_CONFIG.robotToCam.getTranslation())
            .rotateAround(new Translation3d(), new Rotation3d(0,0,pose.get().getRotation().getRadians()))
            .plus(new Translation3d(pose.get().getTranslation().getX(), pose.get().getTranslation().getY(), 0))
            );
        }

        Logger.recordOutput("AScope/detection cam", CAMERA_CONFIG.robotToCam.getTranslation()
            .rotateAround(new Translation3d(), new Rotation3d(0,0,pose.get().getRotation().getRadians()))
            .plus(new Translation3d(pose.get().getTranslation().getX(), pose.get().getTranslation().getY(), 0)));

        Logger.recordOutput("Subsystems/Fuel Vision/area estimates", areaEstimates.stream().mapToDouble(e->e).toArray());
        Logger.recordOutput("Subsystems/Fuel Vision/radial estimates", radialEstimates.toArray(new Translation3d[0]));
        Logger.recordOutput("Subsystems/Fuel Vision/distance estimates", distEstimates.stream().mapToDouble(e->e).toArray());
        Logger.recordOutput("Subsystems/Fuel Vision/height estimates", heightEstimates.stream().mapToDouble(e->e).toArray());
        Logger.recordOutput("Subsystems/Fuel Vision/horizantal dist estimates", hDistEstimates.stream().mapToDouble(e->e).toArray());
        Logger.recordOutput("Subsystems/Fuel Vision/theta estimates", thetaEstimates.stream().mapToDouble(e->e).toArray());
    }
}
