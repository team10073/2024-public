package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.PrimerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.PrimerConstants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;


/**
 * Class for PrimerSubsystem to move game pieces from indexer to a holding space near the shooter.
 */
public class PrimerSubsystem extends SubsystemBase{
  /** CANSparkMax motor controller for the priming roller. */
  private TalonSRX primerNeo;
  private AnalogInput speakerBeamBreak;
  public boolean primerStow;

  /** CANSparkMan pid controller */
  //private SparkPIDController pidController;

  /**
   * Creates a new PrimerSubsystem with initialized motor controller.
   */
  public PrimerSubsystem() {
    primerNeo = new TalonSRX(PrimerConstants.kPrimerRollerMotorID);
    primerNeo.setInverted(true);
    primerNeo.setNeutralMode(NeutralMode.Brake);
    primerNeo.configVoltageCompSaturation(12);
    primerNeo.enableVoltageCompensation(true);
    speakerBeamBreak = new AnalogInput(ShooterConstants.kShooterAnalogInputChannel);
    // pidController.setP(PrimerConstants.kP);
    // pidController.setI(PrimerConstants.kI);
    // pidController.setFF(PrimerConstants.kFF);

  }

  /**
   * Sets the priming roller speed for priming the shooting mechanism.
   */
  public void primeNote(double rpm) {
    //pidController.setReference(rpm, ControlType.kVelocity);
  }
  
  public void setSpeed(double speed) {
    primerNeo.set(ControlMode.PercentOutput,speed);
  }
  /**
   * Run the motor backwards at a slow speed of kPrimerReverseSpeed.
   */
  
  public void returnNote() {
    primerNeo.set(ControlMode.PercentOutput,PrimerConstants.kOuttake);
  }
  public void note() {
    primerNeo.set(ControlMode.PercentOutput,PrimerConstants.kIntake);
  }
  /**
   * Stops the primer motor
   */
  public void stop() {
    primerNeo.set(ControlMode.PercentOutput,PrimerConstants.kStop);
  }

  /**
   * Checks if the game piece is pinched into the primer at a certain point.
   * @return true or false if game piece is held well in primer.
   */
  public boolean isObjectPinchedInPrimer() {
    return (isPrimerBeamBreakBroken());
  }

  /** Returns the velocity of the motor */
  // public double getVelocity() {
  //   return pidController.getSmartMotionMaxVelocity(PrimerConstants.kPrimerSlotID);
  // }



  public Command ampCommand() {
    return setSpeedCommand(PrimerConstants.kAmp);
  }

  /**
   * Runs the primer forward until the beambreak is broken.
   * <p> Ends on: 
   * <ul>
   *   <li> Beambreak broken
   * </ul>
   * <p> End Behavior: 
   * <ul>
   *   <li> Whether interrupted or not, sets speed to 0 upon finishing.
   * </ul>
   * @return Command
   */
  public Command intakeCommand() {
    if (Utils.isSimulation()) {
      return new WaitCommand(2.5);
    }
    return setSpeedCommand(PrimerConstants.kIntake).andThen(new WaitUntilCommand(this::isPrimerBeamBreakBroken)).finallyDo(() -> setSpeed(0));
  }

  public Command fastIntakeCommand() {
    return setSpeedCommand(.5).andThen(new WaitUntilCommand(this::isPrimerBeamBreakBroken)).andThen(backupCommand());
  }
  
  public Command setIntakeSpeed() {
    return runOnce(() -> setSpeed(PrimerConstants.kIntake));
  }

  public Command outtakeCommand() {
    return setSpeedCommand(PrimerConstants.kOuttake).finallyDo(() -> setSpeed(0));
  }
  public Command shootCommand() {
    return setSpeedCommand(PrimerConstants.kShoot).finallyDo(() -> setSpeed(0));
  }
  public Command stopCommand() {
    return setSpeedCommand(PrimerConstants.kStop);
  }

  public Command setOuttakeSpeed() {
    return runOnce(() -> setSpeed(PrimerConstants.kOuttake));
  }

  public Command backupCommand() {
    return runOnce(() -> setSpeed(-.3)).andThen(new WaitUntilCommand(() -> isPrimerBeamBreakBroken()).withTimeout(0.15)).finallyDo(() -> setSpeed(0));
  }

  public Command setSpeedCommand(double speed) {
    return runOnce(() -> setSpeed(speed));
  }
  public boolean isPrimerBeamBreakBroken() { //To change 
    return ((Math.floor(speakerBeamBreak.getVoltage()) == 0));
}
  @Override
  public void periodic() {
    //System.out.println(isObjectPinchedInPrimer()); //To change
    SmartDashboard.putNumber("Primer output", primerNeo.getMotorOutputVoltage());
    if (primerStow && isPrimerBeamBreakBroken()) {
      //setSpeed(PrimerConstants.kIntake);
    }
  }
  
}
