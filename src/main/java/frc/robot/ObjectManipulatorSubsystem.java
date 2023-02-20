package frc.robot;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectManipulatorSubsystem extends SubsystemBase{
    
    // define motors to be used
    private final CANSparkMax elevatorNeo = new CANSparkMax(Constants.ELEVATOR_ID, MotorType.kBrushless);
    private final CANSparkMax extendorNeo = new CANSparkMax(Constants.EXTENDOR_ID, MotorType.kBrushless);

    // get encoders from motors
    private final RelativeEncoder elevatorEncoder = elevatorNeo.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    private final RelativeEncoder extendorEncoder = extendorNeo.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    // values for feedforward to be determined by SysID toolsuite, using elevator tool
    private final SimpleMotorFeedforward elevatorFeedforwardController = new SimpleMotorFeedforward(0 * Constants.VOLTAGE_TO_PERCENT_POWER, 0 * Constants.VOLTAGE_TO_PERCENT_POWER);
    
    // values for pid controllers determined experimantally
    private final ProfiledPIDController elevatorPIDController = 
        new ProfiledPIDController(
            0.0,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                1, 5));

    
    private final PIDController extendorPIDController = new PIDController(0, 0, 0);
    

    // define solenoids to be used for pistons
    private final DoubleSolenoid pincherSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PINCHER_SOLENOID_FORWARD, Constants.PINCHER_SOLENOID_REVERSE);
    private final DoubleSolenoid pivotSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PIVOT_SOLENOID_FORWARD, Constants.PIVOT_SOLENOID_REVERSE);

    //define DIO ports for limit switches for top (and possibly bottom) of the elevator
    private final DigitalInput topLimitSwitch = new DigitalInput(Constants.TOP_LIMIT_SWITCH_DIO_PIN);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(Constants.BOTTOM_LIMIT_SWITCH_DIO_PIN);

    public ObjectManipulatorSubsystem(){
        //register();
        extendorNeo.setIdleMode(IdleMode.kBrake);
    }

    
    // piston actions
    public void closePincher(){
        pincherSolenoid.set(DoubleSolenoid.Value.kReverse);    
    }

    public void openPincher(){
        pincherSolenoid.set(DoubleSolenoid.Value.kForward);    
    }

    public void pivotBack(){
        pivotSolenoid.set(DoubleSolenoid.Value.kReverse);    
    }

    public void pivotForward(){
        pivotSolenoid.set(DoubleSolenoid.Value.kForward);    
    }

    // These setpoints are in meters from their resting position
    private double elevatorSetPoint = Constants.ELEVATOR_MAX + 0.1;
    private double extendorSetPoint = 0;

    public boolean manualElevatorControl = false;

    public void increaseElevator(double amount){
        //if(elevatorSetPoint + amount < Constants.ELEVATOR_MAX && elevatorSetPoint + amount > Constants.ELEVATOR_MIN){
        //    elevatorSetPoint += amount;
        //}
        
        if(manualElevatorControl){
            elevatorNeo.set(-amount);
        }
        
    }

    public void manualExtend(double amount){
        extendorNeo.set(amount);
    }

    
    /**
     * sets elevator setpoint
     * @param height Elevator height in m, where 0 is all the way lowered
     */
    public void setElevator(double height){
        elevatorSetPoint = height;
        manualElevatorControl = false;
    }

    /**
     * sets extendor setpoint
     * @param distance Extension distance in m from its starting configuration, which is all the way retracted
     */
    public void setExtendor(double distance){
        extendorSetPoint = distance;
    }


    public boolean pincherClosed(){
        boolean closed = true;
        if(pincherSolenoid.get() == Value.kForward){
            closed = false;
        }
        return closed;
    }

    public boolean pivotRetracted(){
        boolean retracted = true;
        if(pivotSolenoid.get() == Value.kReverse){
            retracted = false;
        }
        return retracted;
    }
    /**
     * returns the currently measured elevator height in m, with 0 being all the way down
     */
    public double getHeight(){
        double height = Constants.ELEVATOR_START_HEIGHT;
        height += -elevatorEncoder.getPosition() * (1.0/11.2) * 0.1333; // gearing ratio is currently 12:1
        // TODO: add another constant multiplier for converting rotations to chain length movement
        return height;
    }

    /**
     * returns the currently measured extension distance in m, with 0 being all the way retracted
     */
    public double getExtension(){
        double extension = Constants.ELEVATOR_START_HEIGHT;
        extension += extendorEncoder.getPosition() * (1.0/4.0); // gearing ratio is currently 4:1
        // TODO: add another constant multiplier for converting rotations to chain length movement
        return extension;
    }

    @Override
    public void periodic(){

        // calculate elevator control output
        double elevatorPID = elevatorPIDController.calculate(getHeight(), elevatorSetPoint);
        double elevatorFeedforward = elevatorFeedforwardController.calculate(elevatorPIDController.getSetpoint().velocity);

        double extendorPID = extendorPIDController.calculate(getExtension(), extendorSetPoint);
        // uncomment this when ready to actually run the mechanism

        double elevatorCommand = 0;
        if(elevatorSetPoint > getHeight()){
            elevatorCommand = 0.7;
            if(getHeight() > Constants.ELEVATOR_MAX - Constants.ELEVATOR_SLOW){
                elevatorCommand = 0.5;
            }
            if(getTopLimitSwitch()){
                elevatorCommand = 0.0;
            }
        }else if(elevatorSetPoint < getHeight()){
            elevatorCommand  = -0.5;
            if(getHeight() < Constants.ELEVATOR_MIN + Constants.ELEVATOR_SLOW){
                elevatorCommand = -0.3;
            }
            if(getBottomLimitSwitch()){
                elevatorCommand = 0.0;
            }
        }

        if(!manualElevatorControl){
            elevatorNeo.set(-elevatorCommand);
        }
        
        //elevatorNeo.set(elevatorFeedforward + elevatorPID);
        //extendorNeo.set(extendorPID);
        
        // print useful information
        SmartDashboard.putNumber("Elevator setpoint", elevatorSetPoint);
        SmartDashboard.putNumber("Elevator Height", getHeight());
        SmartDashboard.putNumber("Extension Distance", getExtension());
        SmartDashboard.putBoolean("Top Limit Switch State", getTopLimitSwitch());
        SmartDashboard.putBoolean("Bottom Limit Switch State", getBottomLimitSwitch());
        
    }

    //returns true if the limit switch at the top of the elevator is pressed
    public boolean getTopLimitSwitch(){
        return topLimitSwitch.get();
    }
    
    //returns true if the limit switch at the bottom of the elevator is pressed
    public boolean getBottomLimitSwitch(){
        return bottomLimitSwitch.get();
    }

}
