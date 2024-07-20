package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterWristConstants;



public class ShooterPivotSubsystem extends SubsystemBase{
    private TalonFXConfiguration encoder;

    public TalonFX master;
    private TalonFX slave;

    private double targetPose = ShooterWristConstants.kIntakePos;
  
    public ShooterPivotSubsystem() {
      TalonFXConfiguration configs = new TalonFXConfiguration();
      master = new TalonFX(ShooterWristConstants.kShooterMasterID);
      slave = new TalonFX(ShooterWristConstants.kShooterSlaveID);
      slave.setControl(new Follower(ShooterWristConstants.kShooterMasterID, true));
      configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      configs.CurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(50);
      configs.Feedback.FeedbackRemoteSensorID = 50;//to change
      configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      Slot0Configs pidController = configs.Slot0;
      pidController.kP = ShooterWristConstants.kP;
      pidController.kD = ShooterWristConstants.kD;
      pidController.kS = ShooterWristConstants.kS;
      master.get(); 
      master.getConfigurator().apply(configs);
      slave.getConfigurator().apply(configs);

      //master.enableVoltageCompensation(12);
      //slave.enableVoltageCompensation(12);


      SmartDashboard.putNumber("target", targetPose);
      
      setRequest(master.getPosition().getValueAsDouble());
    }
    public void test() {
        master.set(0.1);
    }

    public void setRequest(double position) {
        targetPose = position;
       

    }
    public boolean atPosition(double position) {
        return (Math.abs(master.getPosition().getValueAsDouble()- position) < ShooterWristConstants.kTolerance);
    }

    public boolean atSetpoint() {
        return (Math.abs(master.getPosition().getValueAsDouble() - targetPose) < ShooterWristConstants.kTolerance); 
    }
 
    public double getTargetPose(){
        return targetPose;
    }

    public void stop() {
        master.set(0);
    }

    public Command runPivot(double position) {
        return runOnce(() -> setRequest(position)).andThen(new WaitUntilCommand(() -> atPosition(position)));
    }

    public Command goToAmpPose(){
        return runPivot(ShooterWristConstants.kAmpPos);
    }

    /**
     * Sets the targetPose variable to indexing position and the waits until pivot is at that position.
     * 
     * <p> Ends on: pivot within tolerance
     * 
     * <p> End behavior: The pivot stop if it is within tolerance,
     * otherwise the pid will run regardless of whether or not the command is still running.
     * The pid will renable without a new command if the pivot falls out of tolerance.
     * @return SequentialCommandGroup
     */
    public Command goToIntakePos() {
        return runPivot(ShooterWristConstants.kIntakePos);
    }
    public Command goToPodiumPos() {
        return runPivot(ShooterWristConstants.kPodiumPos);
    }
    public Command goToSubCommand() {
        return runPivot(ShooterWristConstants.kSubwooferPos);
    }
    public Command gotToStowCommand() {
        return runPivot(ShooterWristConstants.kStowpos);
    }
    public Command goToParallelPos() {
        return runPivot(ShooterWristConstants.kParallelPos);
    }
    public Command stopCommand() {
        return new InstantCommand(() -> stop());
    }
    public Command testShooter() {
        return run(() -> test()).finallyDo(() -> master.set(0));
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("TargetPose", targetPose);
        SmartDashboard.putNumber("CurrentPose", master.getPosition().getValueAsDouble());
        if (!atSetpoint()){
            master.setControl(new PositionVoltage(targetPose));
        } else {
            master.stopMotor();
        }
       }
    }
    
  
