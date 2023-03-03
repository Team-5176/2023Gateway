package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Auto extends CommandBase{
    //private Drivetrain m_swerve;
    private ObjectManipulatorSubsystem manipulator;
    private int route = 0;

    private Timer timeMan = new Timer();
    private double stateStartTime;
    private boolean isFinished = false;
    
    public Auto(ObjectManipulatorSubsystem m, int pathNumber){
        
        manipulator = m;
        route = pathNumber;
        addRequirements(m);
    }

    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    

    @Override
    public void initialize(){
        timeMan.start();
        stateStartTime = timeMan.get();
        if(Robot.m_swerve == null){
            //initiate swerve based on starting position specified in SmartDashboard. I rounded before converting to int just in case there are any double shenanigans
            //m_swerve = new Drivetrain()
            Robot.m_swerve = new Drivetrain(Constants.AutonomousPaths.examplePath.getInitialHolonomicPose());
            
            SmartDashboard.putNumber("Initial Angle", Constants.AutonomousPaths.examplePath.getInitialHolonomicPose().getRotation().getDegrees());
        }
        //Robot.m_swerve.startingHeading = Constants.AutonomousPaths.examplePath.getInitialHolonomicPose().getRotation().getDegrees();
        //m_swerve = Robot.m_swerve;
        Robot.m_swerve.navx.reset();
        //Robot.m_swerve.navx.setAngleAdjustment(route);
        //SmartDashboard.putNumber("navx raw heading", Robot.m_swerve.navx.getAngle());
        Robot.m_swerve.xController.reset();
        Robot.m_swerve.yController.reset();
        Robot.m_swerve.rotController.reset();
        manipulator.extesnionOffset += manipulator.getExtension();

        //configure manipulator
        manipulator.setExtendor(10.50);
        manipulator.closePincher();
        manipulator.pivotForward();
        manipulator.setElevator(Constants.ELEVATOR_MAX + 3);

    }

    
    public void setRoute(int r){
        route = r;
    }

    int state = 0;
    
    

    private boolean started = false;
    @Override
    public void execute(){
        if(!started){
            SmartDashboard.putNumber("time error", timeMan.get() - stateStartTime);
            stateStartTime = timeMan.get();
            started = true;
        }
        SmartDashboard.putNumber("Pos x", Robot.m_swerve.getPose().getX());
        SmartDashboard.putNumber("Pos y", Robot.m_swerve.getPose().getY());
        SmartDashboard.putNumber("Heading", Robot.m_swerve.getHeading());
        SmartDashboard.putNumber("navx raw heading", Robot.m_swerve.navx.getAngle());
        
        if(Constants.AUTO == 0){
            auto1();
        }

        
        if(!Robot.m_swerve.navx.isConnected()){
            isFinished = true;
        }
    }
    private void auto1(){
        if(state == 0){
            if(timeMan.get() - stateStartTime > 3.0){
                state ++;
                manipulator.openPincher();
                
                stateStartTime = timeMan.get();
            }
        }
        else if(state == 1){
            if(timeMan.get() - stateStartTime > .5){
                manipulator.setExtendor(3);
                state ++;
                stateStartTime = timeMan.get();
            }
        }
        else if (state == 2){
            Robot.m_swerve.matchPath((PathPlannerState)Constants.AutonomousPaths.examplePath.sample(timeMan.get() - stateStartTime));
            if(timeMan.get() - stateStartTime > 2.0){
                manipulator.setElevator(Constants.ELEVATOR_MIN - 2);
                manipulator.pivotBack();
            }
            if(Constants.AutonomousPaths.examplePath.getTotalTimeSeconds() < timeMan.get() - stateStartTime){
                manipulator.closePincher();
                state += 1;
                stateStartTime = timeMan.get();
            }
        }else if(state == 3){
            Robot.m_swerve.matchPath((PathPlannerState)Constants.AutonomousPaths.path1_2.sample(timeMan.get() - stateStartTime));
            if(timeMan.get() - stateStartTime > 2){
                manipulator.setElevator(Constants.ELEVATOR_MAX + 2);
                manipulator.pivotForward();
                manipulator.setExtendor(10.0);
            }
            if(Constants.AutonomousPaths.path1_2.getTotalTimeSeconds() < timeMan.get() - stateStartTime){
                manipulator.openPincher();
                state ++;
                stateStartTime = timeMan.get();
            }
        }
         else{
            isFinished = true;
        }
        //
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
