package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Auto extends CommandBase{
    private Drivetrain m_swerve;
    private ObjectManipulatorSubsystem manipulator;
    private int route = 0;

    private Timer timeMan = new Timer();
    private double stateStartTime;
    
    public Auto(ObjectManipulatorSubsystem m, Drivetrain d){
        m_swerve = d;
        manipulator = m;
        
        addRequirements(m, d);
    }

    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new PathConstraints(3, 2));

    @Override
    public void initialize(){
        timeMan.start();
        stateStartTime = timeMan.get();
        //m_swerve.m_poseEstimator.addVisionMeasurement(examplePath.getInitialHolonomicPose(), Timer.getFPGATimestamp(), VecBuilder.fill(0.0, 0.0, 0.0));
    }

    
    public void setRoute(int r){
        route = r;
    }

    int state = 0;
    
    private void auto1(){
        
    }

    @Override
    public void execute(){
        m_swerve.matchPath((PathPlannerState)examplePath.sample(timeMan.get() - stateStartTime));
    }

}
