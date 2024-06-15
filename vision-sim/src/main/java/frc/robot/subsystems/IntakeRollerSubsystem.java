package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeWristConstants;
import frc.robot.Constants.IntakeRollerConstants;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

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

    /** PID controller for maintaining the turning motor position. */
    SparkPIDController pidController;

    /** Flag indicating whether the intake is outside or not. */
    boolean outside;
    private AnalogInput rollerSensor;
 

    double buttonLastTrigger = 0;

    /**
     * Creates a new IntakeSubsystem with initialized motor controllers and PID
     * controller.
     */
    public IntakeRollerSubsystem() {

        // Initialization of motor controllers and PID controller
        rollerMotor = new TalonFX(IntakeRollerConstants.kIntakeRollerID);
        
        rollerSensor = new AnalogInput(IntakeRollerConstants.kIntakeAnalogInputChannel);
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.CurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(IntakeRollerConstants.kIntakeCurrentLimit);
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        rollerMotor.getConfigurator().apply(configs);
    }

    
    public void setSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public void stop() {
        rollerMotor.set(0); // I would use stop motor, but it hasn't been tested and I don't want any weird behavior.
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
    public Command intakeSenseCommand() {
        return setSpeedCommand(IntakeRollerConstants.kIntake).withTimeout(5).until(() -> intakeHasPiece()).finallyDo(() -> setSpeed(0));
    }
    public Command dumbIntakeCommand(){
        return setSpeedCommand(IntakeRollerConstants.kIntake).withTimeout(.4);
    }
    public Command intakeCommand(){
        return intakeSpeedCommand().until(() -> intakeHasPiece()).finallyDo(() -> setSpeed(0));
    }
    public Command holdCommand() {
        return setSpeedCommand(IntakeRollerConstants.kHold);
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

    public Command stowSpeedCommand() {
        return runOnce(() -> setSpeed(IntakeRollerConstants.kHold));
    }
    public Command stopCommand() {
        return runOnce(() -> setSpeed(0));
    }

    /**
     * Run once intake is down only
     * @return
     */
    public boolean intakeHasPiece() {
        return Timer.getFPGATimestamp() - buttonLastTrigger > 0.125;
    }

    public boolean buttonPressed() {
        return rollerSensor.getVoltage() <= 0.2;
    }

    

    
    @Override
    public void periodic() {
        if (!buttonPressed()) {
            buttonLastTrigger = Timer.getFPGATimestamp();
        }
        SmartDashboard.putNumber("roller Voltage", rollerSensor.getVoltage());
    }
}