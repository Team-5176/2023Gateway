package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManipulatorCommand extends CommandBase{
    
    public ManipulatorCommand(){
        
    }

    public int designateStep = 0;
    private int[] designations = {0, 0, 0};
    private boolean runningPath;
    private PathPlannerTrajectory scoringTrajectory;
    private double pathStartTime;

    @Override
    public void initialize(){
        designateStep = 0;
        runningPath = false;
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("test???", 74);
        SmartDashboard.putNumber("Designate step", designateStep);

        if(Robot.m_controller.getRawButton(Constants.ELEVATOR_UP)){

        }
        else if(Robot.m_controller.getRawButton(Constants.ELEVATOR_DOWN)){
           
        }
        else{
             
        }

        if(Robot.m_controller.getRawButton(Constants.CLOSE_GRABBER)){
            
        }
        if(Robot.m_controller.getRawButton(Constants.OPEN_GRABBER)){
            
        }
        if(Robot.m_controller.getRawButton(Constants.PIVOT_BACK)){
            
        }
        if(Robot.m_controller.getRawButton(Constants.PIVOT_FORWARD)){
            
        }
        if(Robot.m_copilot_controller.getRawButton(Constants.EXTEND)){
            
        }
        else if(Robot.m_copilot_controller.getRawButton(Constants.RETRACT)){
            
        } else{
            
        }
        if(Robot.m_copilot_controller.getRawButton(Constants.MOVE_ELEVATOR_MAX)){
            
        }
        if(Robot.m_copilot_controller.getRawButton(Constants.MOVE_ELEVATOR_MIN)){
            
        }
        if(Robot.m_copilot_controller.getRawButton(Constants.STOW_ELEVATOR)){
            
        }
        
        if(designateStep <= 1){
            if(Robot.m_copilot_controller.getRawButtonReleased(Constants.DESIGNATE_LEFT)){
                designations[designateStep] = 0;
                designateStep ++;
            }
            if(Robot.m_copilot_controller.getRawButtonReleased(Constants.DESIGNATE_RIGHT)){
                designations[designateStep] = 2;
                designateStep ++;
            }
            if(Robot.m_copilot_controller.getRawButtonReleased(Constants.DESIGNATE_UP)){
                designations[designateStep] = 1;
                designateStep ++;
            }
            if(Robot.m_copilot_controller.getRawButtonReleased(Constants.DESIGNATE_DOWN)){
                designations[designateStep] = 1;
                designateStep ++;
            }
        }
        else if(designateStep == 2){
            if(Robot.m_copilot_controller.getRawButtonReleased(Constants.DESIGNATE_LEFT)){
                designations[designateStep] = 0;
                designateStep ++;
            }
            if(Robot.m_copilot_controller.getRawButtonReleased(Constants.DESIGNATE_RIGHT)){
                designations[designateStep] = 0;
                designateStep ++;
            }
            if(Robot.m_copilot_controller.getRawButtonReleased(Constants.DESIGNATE_UP)){
                designations[designateStep] = 1;
                designateStep ++;
            }
            if(Robot.m_copilot_controller.getRawButtonReleased(Constants.DESIGNATE_DOWN)){
                designations[designateStep] = -1;
                designateStep ++;
            }
        }
        if(Robot.m_copilot_controller.getRawButtonReleased(Constants.DESIGNATE_RESET)){
            designateStep = 0;
        }

        if(Robot.m_controller.getRawButtonPressed(Constants.EXECUTE_AUTO)){
            runningPath = true;
            Pose2d targetPose;
            if(Constants.IS_BLUE){
                targetPose = Constants.ScoringPositions.blueScorePoints[designations[0]*3 + designations[1]];
            } else{
                targetPose = Constants.ScoringPositions.redScorePoints[designations[0]*3 + designations[1]];
            }
            Pose2d velocity = Robot.m_swerve.getVelocity();
            double speed = Math.sqrt(velocity.getX()*velocity.getX() + velocity.getY()*velocity.getY());
            scoringTrajectory = PathPlanner.generatePath(
                new PathConstraints(3, 2), 
                new PathPoint(Robot.m_swerve.getPose().getTranslation(), velocity.getRotation(), Robot.m_swerve.getPose().getRotation(), speed), // position, heading(direction of travel), holonomic rotation, velocity override
                new PathPoint(targetPose.getTranslation(), targetPose.getRotation(), targetPose.getRotation()) // position, heading(direction of travel), holonomic rotation
            );
            if(designations[2] == -1){
                
            }
            else if(designations[2] == 0){
                
            }
            else if(designations[2] == 1){
                
            }
            pathStartTime = Timer.getFPGATimestamp();
        }

        if(runningPath){
            Robot.m_swerve.matchPath((PathPlannerState)scoringTrajectory.sample(Timer.getFPGATimestamp() - pathStartTime));
            if(scoringTrajectory.getTotalTimeSeconds() < Timer.getFPGATimestamp() - pathStartTime){
                runningPath = false;
            }
            if(Robot.m_controller.getRawButton(Constants.STOP_AUTO)){
                runningPath = false;
            }
        }


        /* 
        int pov = Robot.m_copilot_controller.getPOV();
        SmartDashboard.putNumber("POV", pov);
        if(Robot.m_copilot_controller.getRawButtonReleased(Constants.DESIGNATE_LEFT)){

        }
        */

        //$if(Robot.m_controller)
    }
}
