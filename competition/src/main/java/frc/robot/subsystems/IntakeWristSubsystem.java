package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeWristConstants;

public class IntakeWristSubsystem extends SubsystemBase{
    /** Motor controller for turning the intake mechanism. */
    TalonFX turningMotor;

    

    /** The required position for the turning motor. */
    double reqPosition;
    double current;

    /**
     * Creates a new IntakeSubsystem with initialized motor controllers and PID
     * controller.
     */
    public IntakeWristSubsystem() {

        // Initialization of motor controllers and PID controller
        turningMotor = new TalonFX(IntakeWristConstants.kIntakeTurnID);
        
        TalonFXConfiguration talonFxConfig = new TalonFXConfiguration();

        Slot0Configs pidController = talonFxConfig.Slot0;
        pidController.kP = IntakeWristConstants.kP;
        pidController.kD = IntakeWristConstants.kFF;
        

        // MotionMagicConfigs motionMagic = talonFxConfig.MotionMagic; // tune all of this
        // motionMagic.MotionMagicCruiseVelocity = ShooterConstants.kMotionMagicCruiseVelocity;
        // motionMagic.MotionMagicAcceleration = ShooterConstants.kMotionMagicCruiseAcceleration;
        // motionMagic.MotionMagicJerk = ShooterConstants.kMotionMagicJerk;
        talonFxConfig.CurrentLimits
            .withStatorCurrentLimit(IntakeWristConstants.kStatorLimit)
            .withSupplyCurrentLimit(IntakeWristConstants.kSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        
        turningMotor.getConfigurator().apply(talonFxConfig);
        turningMotor.setNeutralMode(NeutralModeValue.Brake);
        

        // Setting the initial required position to the origin
        turningMotor.setPosition(IntakeWristConstants.kStow);

    }

    public void testIntake() {
        turningMotor.set(0.1);
    }

    public void stopIntake() {
        turningMotor.set(0);
    }
    
    /**
     * Sets the target position for the turning motor using PID control.
     *
     * @param req The target position for the turning motor.
     */
    public void intakeToReq(double req) { 
        turningMotor.setControl(new PositionDutyCycle(req));  
    }

    /**
     * Sets the required position for the turning motor.
     *
     * @param req The required position for the turning motor.
     */
    public void setRequest(double req) {
        reqPosition = req;
        intakeToReq(reqPosition);
    }

    /**
     * Gets the current position of the turning motor.
     *
     * @return The current position of the turning motor.
     */
    public double getPosition() {
        return turningMotor.getPosition().getValue();
    }

    public void stop() {
        turningMotor.setControl(new NeutralOut());
    }

    /**
     * Checks if the turning motor is at the required position within a specified
     * tolerance.
     *
     * @param reqPos The required position to check against.
     * @return True if the turning motor is at the required position; false
     *         otherwise.
     */
    public boolean isAtReqPosition(double reqPos) {

        if ((getPosition() >= (reqPos - IntakeWristConstants.kTolerance))
                && (getPosition() <= (reqPos + IntakeWristConstants.kTolerance))) {
            return true;
        }
        return false;
    }

    public Command indexPosCommand() {
        return runOnce(() -> intakeToReq(IntakeWristConstants.kStow)).andThen(new WaitUntilCommand(() ->  isAtReqPosition(IntakeWristConstants.kStow))).finallyDo(() -> stop());
    }
    public Command intakePosCommand() {
        return runOnce(() -> intakeToReq(IntakeWristConstants.kIntake)).andThen(new WaitUntilCommand(() ->  isAtReqPosition(IntakeWristConstants.kIntake))).finallyDo(() -> stop());
    }
    public Command ampPosCommand() {
        return runOnce(() -> intakeToReq(IntakeWristConstants.kAmp)).andThen(new WaitUntilCommand(() ->  isAtReqPosition(IntakeWristConstants.kAmp))).finallyDo(() -> stop());
    }
    public Command stopMotorCommand(){
        return runOnce(this::stop);
    }
    /**
     * Ewww ugly get this out of here
     * @return bad
     */
    public Command intakeTest() {
        //return run(this::testIntake).finallyDo(() -> stopIntake());
        return new SequentialCommandGroup(runOnce(() -> intakeToReq(IntakeWristConstants.kIntake)).andThen(new WaitUntilCommand(() -> isAtReqPosition(IntakeWristConstants.kIntake))));
    }
  

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //System.out.println(getPosition());
        SmartDashboard.putNumber("Pose", getPosition());
        SmartDashboard.putNumber("Current", current);
        
    }
}
