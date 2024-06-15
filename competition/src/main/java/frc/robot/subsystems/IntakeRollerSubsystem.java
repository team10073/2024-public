package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.IntakeRollerConstants;
import com.revrobotics.SparkPIDController;

/**
 * The IntakeSubsystem class represents a subsystem for controlling the intake
 * mechanism of a robot.
 * It includes functionality for intake and outtake operations, positioning the
 * intake, retrieving the current position,
 * checking if the intake is at the required position, and detecting the
 * presence of an object on the intake.
 */
public class IntakeRollerSubsystem extends SubsystemBase {
    /** Motor controller for controlling the intake. */
    TalonFX rollerMotor;

    /** Flag indicating whether the intake is outside or not. */
    boolean outside;
    private AnalogInput rollerSensor;
 

    double buttonLastTrigger = 0;

    double intakingSpeed = IntakeRollerConstants.kIntake;

    VoltageOut voltage = new VoltageOut(0);

    boolean isDebug = false;

    Mechanism2d intakeButtonStatus;
    

    /**
     * Creates a new IntakeSubsystem with initialized motor controllers and PID
     * controller.
     */
    public IntakeRollerSubsystem() {

        // Initialization of motor controllers and PID controller
        rollerMotor = new TalonFX(IntakeRollerConstants.kIntakeRollerID);
        intakeButtonStatus = new Mechanism2d(500, 500);
        
        rollerSensor = new AnalogInput(IntakeRollerConstants.kIntakeAnalogInputChannel);
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(IntakeRollerConstants.kStatorLimit)
            .withSupplyCurrentLimit(IntakeRollerConstants.kSupplyLimit)
            // .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);

        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        rollerMotor.getConfigurator().apply(configs);

        if (isDebug) {
            configureDebug();
        }
    }

    private void configureDebug() {
        SmartDashboard.putNumber("Intake speed", intakingSpeed);
    }

    
    public void setSpeed(double speed) {
        rollerMotor.setControl(voltage.withOutput(speed * 12));
    }

    public void stop() {
        rollerMotor.setControl(new NeutralOut()); // I would use stop motor, but it hasn't been tested and I don't want any weird behavior.
    }

    /**
     * Checks if an object is detected on the intake based on the current output
     * current.
     *
     * @return True if an object is detected on the intake; false otherwise.
     */


    /**
     * COMMANDS
     */

    /**
     * Creates an InstantCommand for outtake operation.
     *
     * @return An InstantCommand object for outtake operation.
     */

    
    // Needs to go but idk if it can be replaced with out breaking stuff.
  
    public Command holdSpeedCommand() {
        return runOnce(() -> setSpeed(IntakeRollerConstants.kHold));
    }
    public Command setSpeedCommand(double speed){
        return run(() -> setSpeed(speed)); // needs to be an instant command. MUST FIX AFTER COMP
    }

    public Command intakeSpeedCommand() {
        return runOnce(() -> setSpeed(IntakeRollerConstants.kIntake));
    }

    public Command outtakeCommand() {
        return runOnce(() -> setSpeed(IntakeRollerConstants.kOuttake));
    }
    public Command ampCommand() {
        return runOnce(() -> setSpeed(IntakeRollerConstants.kAmp));
    }

    public Command stowSpeedCommand() {
        return runOnce(() -> setSpeed(IntakeRollerConstants.kHold));
    }
    public Command stopCommand() {
        return runOnce(() -> stop());
    }


    public Command autonIntakeSpeedCommand() {
        return run(() -> setSpeed(IntakeRollerConstants.kIntake));
    }

    /**
     * Run once intake is down only
     * @return
     */
    public boolean intakeHasPiece() {
        
        return Timer.getFPGATimestamp() - buttonLastTrigger > 0.125; //|| rollerMotor.getVelocity().getValueAsDouble() <= 1.0;
    }

    public boolean buttonPressed() {
        return rollerSensor.getVoltage() <= 0.2; 
    }

    

    
    @Override
    public void periodic() {
        if (!buttonPressed()) {
            buttonLastTrigger = Timer.getFPGATimestamp();
        }
        SmartDashboard.putNumber("Intake button voltage", rollerSensor.getVoltage());
        SmartDashboard.putNumber("Roller speed", rollerMotor.getVelocity().getValueAsDouble());
        if (intakeHasPiece()) {
            intakeButtonStatus.setBackgroundColor(new Color8Bit(Color.kGreen));
        }
        else {
            intakeButtonStatus.setBackgroundColor(new Color8Bit(Color.kRed));
        }
        SmartDashboard.putData("Intake has piece", intakeButtonStatus);
    }
}