package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A {@link edu.wpi.first.wpilibj.smartdashboard.SendableChooser} for selecting an autonomous program.
 * <br/><br/>
 * <p>
 * How to add a new autonomous program.
 *     <ol>
 *         <li>
 *             Add a new value to {@link Auto}<br/>
 *             The name of the value should use screaming snake case
 *             </li>
 *             <li>
 *                 Add a new method in {@link AutoFactory} that returns a {@link edu.wpi.first.wpilibj2.command.Command}.
 *             </li>
 *             <li>
 *                 Add a new {@link AutoProgram} to {@link AutoChooser#AUTO_PROGRAMS}.
 *             </li>
 *             <li>
 *                 Implement the autonomous program factory method.
 *             </li>
 *             <li>
 *                 Test, test, and test some more.
 *             </li>
 *         </ol>
 * </p>
 */
public class AutoChooser {

    private AutoFactory factory;

    private Field2d field = new Field2d();

    private static final List<AutoProgram> AUTO_PROGRAMS = List.of(
        // new AutoProgram("example", AutoFactory::createExampleAuto),
            new AutoProgram("idle", AutoFactory::createIdleAuto),
            new AutoProgram("test", AutoFactory::createTestAuto)
    );

    private LoggedDashboardChooser<AutoProgram> chooser;

    public AutoChooser(AutoFactory factory){
        chooser = new LoggedDashboardChooser<>("Auto Program");
        for (int i=0; i<AUTO_PROGRAMS.size(); i++){
            if(i == 0){
                chooser.addDefaultOption(AUTO_PROGRAMS.get(i).getLabel(), AUTO_PROGRAMS.get(i));
            } else {
                chooser.addOption(AUTO_PROGRAMS.get(i).getLabel(), AUTO_PROGRAMS.get(i));
            }
        }
        this.factory = factory;
        
        SmartDashboard.putData("autoPath", field);
    }

    public Command getSelected(){
        return chooser.get().getCommand(factory);
    }

    public void update(){

        field.getObject("traj").setPoses(chooser.get().getPoses(factory));

    }

}