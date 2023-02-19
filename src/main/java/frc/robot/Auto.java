package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Auto extends CommandBase{
    private Drivetrain m_swerve;
    private ObjectManipulatorSubsystem manipulator;
    private int route = 0;

    public Auto(ObjectManipulatorSubsystem m, Drivetrain d){
        m_swerve = d;
        manipulator = m;

        addRequirements(m, d);
    }

    public void setRoute(int r){
        route = r;
    }

    int state = 0;
    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));
    private void auto1(){
        
    }

}
