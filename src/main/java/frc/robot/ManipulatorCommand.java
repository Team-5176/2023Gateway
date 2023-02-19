package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManipulatorCommand extends CommandBase{
    
    public ObjectManipulatorSubsystem manipulator;
    public ManipulatorCommand(ObjectManipulatorSubsystem sub){
        manipulator = sub;
        addRequirements(sub);
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("test???", 74);
        if(Robot.m_controller.getRawButton(Constants.ELEVATOR_UP)){
            manipulator.increaseElevator(0.5);
        }
        else if(Robot.m_controller.getRawButton(Constants.ELEVATOR_DOWN)){
            manipulator.increaseElevator(-0.5);
        }
        else{
            manipulator.increaseElevator(0.0);
        }

        if(Robot.m_controller.getRawButton(Constants.CLOSE_GRABBER)){
            manipulator.closePincher();
        }
        if(Robot.m_controller.getRawButton(Constants.OPEN_GRABBER)){
            manipulator.openPincher();
        }
        if(Robot.m_controller.getRawButton(Constants.PIVOT_BACK)){
            manipulator.pivotBack();
        }
        if(Robot.m_controller.getRawButton(Constants.PIVOT_FORWARD)){
            manipulator.pivotForward();
        }
        if(Robot.m_controller.getRawButton(Constants.EXTEND)){
            manipulator.manualExtend(0.3);
        }
        else if(Robot.m_controller.getRawButton(Constants.RETRACT)){
            manipulator.manualExtend(-0.3);
        } else{
            manipulator.manualExtend(0.0);
        }
        //$if(Robot.m_controller)
    }
}
