// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeWristSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.Constants.IntakeWristConstants;
import frc.robot.Constants.PrimerConstants;
import frc.robot.Constants.ShooterWristConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShootAnywhereCommand;
import frc.robot.commands.handleLEDCommand;
import frc.robot.commands.handlePrimerShooter;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class RobotContainer {
  // SUBSYSTEMS

  private double MaxSpeed = 4.5; // 6 meters per second desired top speed
  private double MaxAngularRate = 4.5; // 3/4 of a rotation per second max angular velocity

  public double power = 4;

  public enum currentMechanism {swerve, intakeWrist, intakeRollers, primer, shooterPivot, shooterRollers};
  currentMechanism sysidMech = currentMechanism.shooterRollers;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  // private final Vision vision = new Vision(drivetrain);
  // private final LEDSubsystem led = new LEDSubsystem();
  // private final ShooterPivotSubsystem pivot = new ShooterPivotSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem(sysidMech);
  // private final IntakeRollerSubsystem intakeRollers = new IntakeRollerSubsystem();
  private final IntakeWristSubsystem intakeWrist = new IntakeWristSubsystem(sysidMech);
  // private final PrimerSubsystem primer = new PrimerSubsystem();
  // private final IndexerSubsystem indexer = new IndexerSubsystem();
  

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * Math.pow(.1, power)).withRotationalDeadband(MaxAngularRate * Math.pow(.1, 4)) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private enum AmpPositionState {Amp, Normal};
  private AmpPositionState ampPosition = AmpPositionState.Normal;
  public SendableChooser<String> autoChooser;
  public Command autonCommand;
  public String currentAuton;
  


  public boolean isIndexing = false;
  public double temp = -1;
  boolean intookPiece;
  boolean stowPivot = false;
  private Direction runDirection;
  
 
  //pathplanner testing
  public RobotContainer() {
        
      
      configureBindings();
      configureDefaultCommands();

      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("Middle subwoofer two piece", "Middle2P");
      autoChooser.addOption("Top subwoofer (Amp side) two piece", "Top2P");
      autoChooser.addOption("Middle subwoofer two piece", "Middle2P");
      autoChooser.addOption("Bottom subwoofer (Source side) two piece", "Bottom2P");
      autoChooser.addOption("Four Piece close. Start center subwoofer", "4 Piece Fixed");
      autoChooser.addOption("Two piece south. Start on source side", "2PSouth");
      autoChooser.addOption("Four piece south. Start on source side", "4PSouthPreload");
      
  }
 


  private void configureBindings() {
    switch (sysidMech) {
      case swerve:
        swerveButtons();
        break;
      case intakeWrist:
        intakeWristButtons();
        break;
      case shooterRollers:
        shooterRollerButtons();
        break;
      default:
        break;
    }
  }

  private void swerveButtons() {
    joystick.a().whileTrue(new ConditionalCommand(drivetrain.sysIdSteerQuasistatic(Direction.kForward), drivetrain.sysIdSteerQuasistatic(Direction.kReverse), () -> runDirection == Direction.kForward));
    joystick.b().whileTrue(new ConditionalCommand(drivetrain.sysIdSteerDynamic(Direction.kForward), drivetrain.sysIdSteerDynamic(Direction.kReverse), () -> runDirection == Direction.kForward));
    joystick.x().whileTrue(new ConditionalCommand(drivetrain.sysIdTranslationQuasistatic(Direction.kForward), drivetrain.sysIdTranslationQuasistatic(Direction.kReverse), () -> runDirection == Direction.kForward));
    joystick.y().whileTrue(new ConditionalCommand(drivetrain.sysIdTranslationDynamic(Direction.kForward), drivetrain.sysIdTranslationDynamic(Direction.kReverse), () -> runDirection == Direction.kForward));
    joystick.leftBumper().whileTrue(new ConditionalCommand(drivetrain.sysIdRotationQuasistatic(Direction.kForward), drivetrain.sysIdRotationQuasistatic(Direction.kReverse), () -> runDirection == Direction.kForward));
    joystick.rightBumper().whileTrue(new ConditionalCommand(drivetrain.sysIdRotationDynamic(Direction.kForward), drivetrain.sysIdRotationDynamic(Direction.kReverse), () -> runDirection == Direction.kForward));
    joystick.rightStick().onTrue(new InstantCommand(() -> runDirection = Direction.kForward));
    joystick.leftStick().onTrue(new InstantCommand(() -> runDirection = Direction.kReverse));
    joystick.start().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(Math.atan2(-joystick.getLeftX(), -joystick.getLeftY())))));
  }

  private void intakeWristButtons() {
    joystick.a().whileTrue(new ConditionalCommand(intakeWrist.sysIdQuasistatic(Direction.kForward), intakeWrist.sysIdQuasistatic(Direction.kReverse), () -> runDirection == Direction.kForward));
    joystick.b().whileTrue(new ConditionalCommand(intakeWrist.sysIdDynamic(Direction.kForward), intakeWrist.sysIdDynamic(Direction.kReverse), () -> runDirection == Direction.kForward));
    joystick.rightStick().onTrue(new InstantCommand(() -> runDirection = Direction.kForward));
    joystick.leftStick().onTrue(new InstantCommand(() -> runDirection = Direction.kReverse));
  }

  private void shooterRollerButtons() {
    joystick.a().whileTrue(new ConditionalCommand(shooter.sysIdQuasistatic(Direction.kForward), shooter.sysIdQuasistatic(Direction.kReverse), () -> runDirection == Direction.kForward));
    joystick.b().whileTrue(new ConditionalCommand(shooter.sysIdDynamic(Direction.kForward), shooter.sysIdDynamic(Direction.kReverse), () -> runDirection == Direction.kForward));
    joystick.povUp().onTrue(new InstantCommand(() -> runDirection = Direction.kForward));
    joystick.povDown().onTrue(new InstantCommand(() -> runDirection = Direction.kReverse));
  }

  public void configureDefaultCommands() {
    //pivot.setDefaultCommand(pivot.goToParallelPos().onlyIf(notInIntake));
     // check wrist up and intake roller beambreak is triggered
  }

  public void setTeleopInitState() {

  }

  public Command getAutonomousCommand() {
    // Command auton = drivetrain.getAutoPath("5Piece");
    //Command baseAuton1 = drivetrain.getAutoPath("Base Auton1");
    //Command middle2P = drivetrain.getAutoPath("Middle2P");
    //Command bottom2P = drivetrain.getAutoPath("Bottom2P");
    // Command baseAuton3 = drivetrain.getAutoPath("Base Auton3");
    //Command theory = drivetrain.getAutoPath("Bottom4P");
    //Command top2Piece = drivetrain.getAutoPath("Top2P");
    //return theory;
    //Command tune = drivetrain.getAutoPath("PathPlanTest");
    //Command baseAuton4 = drivetrain.getAutoPath("4Piece");
    //Command threePieceChoreo = drivetrain.getAutoPath("3 piece");
    //Command fourP = drivetrain.getAutoPath("4P");
    return autonCommand;
  }
}