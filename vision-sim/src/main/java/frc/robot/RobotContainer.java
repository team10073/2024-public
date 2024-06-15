// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeWristSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.Constants.IntakeWristConstants;
import frc.robot.Constants.PrimerConstants;
import frc.robot.Constants.ShooterWristConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShootAnywhereCommand;
import frc.robot.commands.TeleopIntakeToPrimerCommand;
import frc.robot.commands.checkPrimerPiece;
import frc.robot.commands.handleLEDCommand;
import frc.robot.commands.handlePrimerShooter;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.BooleanSupplier;

import javax.swing.GroupLayout.ParallelGroup;
import javax.swing.text.html.HTMLDocument.HTMLReader.IsindexAction;

import frc.robot.subsystems.ArmRollerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class RobotContainer {
  // SUBSYSTEMS

  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  public double power = 4;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Vision vision = new Vision(drivetrain);
  private final LEDSubsystem led = new LEDSubsystem();
  private final ShooterPivotSubsystem pivot = new ShooterPivotSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeRollerSubsystem intakeRollers = new IntakeRollerSubsystem();
  private final IntakeWristSubsystem intakeWrist = new IntakeWristSubsystem();
  private final PrimerSubsystem primer = new PrimerSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  

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
  

  public BooleanSupplier inIntakeUp = (() -> intakeRollers.intakeHasPiece() && intakeWrist.isAtReqPosition(IntakeWristConstants.kStow));
  public BooleanSupplier inIntakeDown = (() -> intakeRollers.intakeHasPiece() && intakeWrist.isAtReqPosition(IntakeWristConstants.kIntake));
  public BooleanSupplier notInIntake = (() -> !intakeRollers.intakeHasPiece() && intakeWrist.isAtReqPosition(IntakeWristConstants.kStow));
  public BooleanSupplier inShooter = (() -> primer.isPrimerBeamBreakBroken());
  public boolean isIndexing = false;
  public double temp = -1;
  boolean intookPiece;
  boolean stowPivot = false;
  
 
  //pathplanner testing
  public RobotContainer() {
      NamedCommands.registerCommand("intakeCommand", new InstantCommand(() -> intookPiece = false).andThen(intakeRollers.intakeSpeedCommand()).andThen(intakeWrist.intakePosCommand()).andThen(new WaitUntilCommand(() -> intakeRollers.intakeHasPiece()).withTimeout(1.5)).finallyDo((interrupted) -> {intookPiece = !interrupted;}));
      NamedCommands.registerCommand("indexCommand", 
        indexer.setForwardSpeedCommand()
        .andThen(intakeWrist.indexPosCommand().alongWith(pivot.goToIntakePos()))
        .andThen(intakeRollers.outtakeCommand())
        .andThen(new SequentialCommandGroup(primer.intakeCommand()).withTimeout(1.5))
        .andThen(intakeRollers.stopCommand().alongWith(indexer.stopCommand())));
      NamedCommands.registerCommand("confirmPiece", new ConditionalCommand(
        new InstantCommand(),
        indexer.setForwardSpeedCommand()
        .andThen(intakeWrist.indexPosCommand().alongWith(pivot.goToIntakePos()))
        .andThen(intakeRollers.outtakeCommand())
        .andThen(new SequentialCommandGroup(primer.intakeCommand())).withTimeout(1.5)
        .andThen(intakeRollers.stopCommand().alongWith(indexer.stopCommand())),
        inShooter));
      NamedCommands.registerCommand("pivotPodium", pivot.runPivot(ShooterWristConstants.kPodiumPos));
      NamedCommands.registerCommand("pivotAmp", pivot.runPivot(ShooterWristConstants.kAmpPos));
      NamedCommands.registerCommand("pivotIntakePos", pivot.runPivot(ShooterWristConstants.kIntakePos));
      NamedCommands.registerCommand("pivotSubwoofer", pivot.runPivot(ShooterWristConstants.kSubwooferPos));
      NamedCommands.registerCommand("primeShooter", new handlePrimerShooter(primer, () -> ampPosition == AmpPositionState.Amp).withTimeout(.375));
      NamedCommands.registerCommand("goToSubwooferSpeed", shooter.setRequestCommand(ShooterConstants.kSubwooferSpeed).andThen(new WaitUntilCommand((() -> shooter.isAtReq())).withTimeout(.5))); // Might want to have a check for is at request instead of just calling this over again.
      NamedCommands.registerCommand("stopShooter", shooter.setRequestCommand(0));
      NamedCommands.registerCommand("fastIndexCommand",
      indexer.setForwardSpeedCommand()
        .andThen(intakeWrist.indexPosCommand().alongWith(pivot.goToIntakePos()))
        .andThen(intakeRollers.outtakeCommand())
        .andThen(new SequentialCommandGroup(primer.fastIntakeCommand()).finallyDo((interrupted) -> {intookPiece = !interrupted;}).withTimeout(.875))
        .andThen(intakeRollers.stopCommand().alongWith(indexer.stopCommand())));
      NamedCommands.registerCommand("confirmShootPiece", new ConditionalCommand(new ParallelCommandGroup(pivot.runPivot(ShooterWristConstants.kSubwooferPos), shooter.setRequestCommand(ShooterConstants.kSubwooferSpeed).andThen(new WaitUntilCommand((() -> shooter.isAtReq())).withTimeout(.5)).andThen(primer.setSpeedCommand(PrimerConstants.kShoot).andThen(new WaitCommand(.375)))), new InstantCommand(), () -> intookPiece));
      NamedCommands.registerCommand("setSubwooferSpeed", new InstantCommand(() -> shooter.setRequest(ShooterConstants.kSubwooferSpeed)));
      NamedCommands.registerCommand("s", new InstantCommand());
      NamedCommands.registerCommand("confirmPieceShort", new ConditionalCommand(
        new InstantCommand(),
        indexer.setForwardSpeedCommand()
        .andThen(intakeWrist.indexPosCommand().alongWith(pivot.goToIntakePos()))
        .andThen(intakeRollers.outtakeCommand())
        .andThen(new SequentialCommandGroup(primer.intakeCommand())).withTimeout(.125)
        .andThen(intakeRollers.stopCommand().alongWith(indexer.stopCommand())),
        inShooter));
      NamedCommands.registerCommand("Eject stuck pieces", 
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            pivot.goToAmpPose(),
            intakeWrist.intakePosCommand()
          ),
          new InstantCommand(() -> {
            primer.setSpeed(PrimerConstants.kAmp);
            intakeRollers.setSpeed(IntakeRollerConstants.kOuttake);
            indexer.setSpeed(IndexerConstants.kForward);
          }),
          new WaitCommand(0.5),
          new InstantCommand(() -> {
            pivot.setRequest(ShooterWristConstants.kIntakePos);
            intakeWrist.setRequest(IntakeWristConstants.kStow);
            primer.stop();
            indexer.stop();
            intakeRollers.stop();
          })
        )
      );
      ShootAnywhereCommand shootAnywhereCommand = new ShootAnywhereCommand(drivetrain, vision, shooter, pivot, led,() -> joystick.getLeftX(), () -> joystick.getLeftY(), () -> joystick.getRightX(), () -> temp);
      // boolean loopForever = true;
      // while (loopForever) {
      //   shootAnywhereCommand.generateValues(300);
      //   try {Thread.sleep(2000);}
      //   catch (Exception e) {
      
      //   }
      // }
      vision.toString();
      //new Trigger(()->DriverStation.isEnabled()).whileTrue(shootAnywhereCommand);
      led.setLedtoIntake();
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
  
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(temp * joystick.getLeftY() * Math.pow(Math.abs(joystick.getLeftY()), power - 1) * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(temp * joystick.getLeftX() * Math.pow(Math.abs(joystick.getLeftX()), power - 1) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * Math.pow(Math.abs(joystick.getRightX()), power - 1) * MaxAngularRate) // Drive counterclockwise with negative X (left)
         ));

    joystick.y().onTrue(
      new SequentialCommandGroup(
        new InstantCommand(() -> {primer.primerStow = false;}), // Currently unnecessary may be used if we need to fix stow idk man
        new ParallelDeadlineGroup(
          intakeWrist.intakePosCommand(),
          intakeRollers.intakeSpeedCommand(),
          pivot.goToIntakePos(),
          new InstantCommand(() -> ampPosition = AmpPositionState.Normal)
        ),
        new WaitUntilCommand(() -> intakeRollers.intakeHasPiece())
          .withTimeout(5),
        intakeRollers.stopCommand(),
        indexer.forwardCommand(),
        intakeWrist.indexPosCommand(),
        new ParallelDeadlineGroup(
          primer.intakeCommand(), // Stops primer by itself
          intakeRollers.outtakeCommand()
        ),
        intakeRollers.stopCommand(),
        indexer.stopCommand(),
        primer.backupCommand()
      )
      .finallyDo(
        () -> {
          intakeRollers.stop();
          indexer.stop();
          if (stowPivot) {
            pivot.setRequest(ShooterWristConstants.kStartPos);
          }
          primer.primerStow = true;
        }
      )
    );
       

    joystick.b().whileTrue(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          pivot.goToAmpPose(),
          intakeWrist.intakePosCommand()
        ),
        new InstantCommand(() -> {primer.primerStow = false;}),
        new StartEndCommand(
          () -> {
            primer.setSpeed(-1);
            intakeRollers.setSpeed(IntakeRollerConstants.kOuttake);
            indexer.setSpeed(IndexerConstants.kForward);
          },
          () -> {
            pivot.setRequest(ShooterWristConstants.kIntakePos);
            intakeWrist.setRequest(IntakeWristConstants.kStow);
            primer.stop();
            indexer.stop();
            intakeRollers.stop();
            primer.primerStow = true;
          }
        )
      )
    );
    // joystick.a().whileTrue(primer.setIntakeSpeed().finallyDo(() -> primer.setSpeed(0)));

    //joystick.a().onTrue(new InstantCommand(() -> {primer.setSpeed(.5);}).andThen(new WaitCommand(.2)).andThen(new InstantCommand(() -> {primer.setSpeed(0);})).andThen(primer.backupCommand()));
    
    joystick.x().onTrue(
      new ParallelCommandGroup(
      new InstantCommand(
        () -> {
          intakeRollers.stop();
          indexer.stop();
          primer.stop();
        }
      ),
      intakeWrist.indexPosCommand()
      )
    );

    joystick.start().onTrue(
      new InstantCommand(
        () -> drivetrain.seedFieldRelative(
          new Pose2d(
            drivetrain.getState().Pose.getTranslation(),
            Rotation2d.fromDegrees(/*(temp == 1) ? 180 : */0)
          )
        )
      )
    );
    //pivot
    joystick.rightBumper().and(() -> primer.isPrimerBeamBreakBroken() || joystick.getHID().getAButtonPressed()).toggleOnTrue(pivot.goToAmpPose()/*.andThen(new WaitCommand(100))*/.alongWith(new InstantCommand(() -> ampPosition = AmpPositionState.Amp))/*.finallyDo(() -> pivot.setRequest(ShooterWristConstants.kStartPos))*/);
    joystick.leftTrigger().and(() -> primer.isPrimerBeamBreakBroken() || joystick.getHID().getAButtonPressed()).whileTrue(pivot.goToPodiumPos().alongWith(new InstantCommand(() -> intakeWrist.setRequest(IntakeWristConstants.kIntake))).alongWith(shooter.setRequestCommand(ShooterConstants.kShoot).alongWith(new WaitUntilCommand(100))).finallyDo(() -> {shooter.stop(); pivot.goToIntakePos(); intakeWrist.setRequest(IntakeWristConstants.kStow);}));
    joystick.leftBumper().and(() -> primer.isPrimerBeamBreakBroken() || joystick.getHID().getAButtonPressed()).whileTrue(pivot.goToSubCommand().alongWith(shooter.setRequestCommand(ShooterConstants.kSubwooferSpeed)).alongWith(new InstantCommand(() -> ampPosition = AmpPositionState.Normal)).alongWith(new WaitCommand(100)).finallyDo(() -> {shooter.stop(); pivot.goToIntakePos();}));
    joystick.rightTrigger().whileTrue(new InstantCommand(() -> {primer.primerStow = false;}).andThen(new handlePrimerShooter(primer,() -> ampPosition == AmpPositionState.Amp)).finallyDo(() -> {primer.primerStow = true;}));
    joystick.rightTrigger().onFalse(primer.backupCommand());
    
    

    
    if (Utils.isSimulation()) {


      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }
  public void configureDefaultCommands() {
    //led.setDefaultCommand(new handleLEDCommand(led, inIntakeDown, inShooter));  
     // check wrist up and intake roller beambreak is triggered
  }

  public void setTeleopInitState() {
    shooter.stop();
    intakeRollers.stop();
    intakeWrist.setRequest(IntakeWristConstants.kStow);
    indexer.stop();
    primer.stop();
    pivot.setRequest(ShooterWristConstants.kIntakePos);
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
    PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("4PSouthSub");
    drivetrain.seedFieldRelative(new Pose2d(path.getPreviewStartingHolonomicPose().getTranslation(), Rotation2d.fromDegrees(-60)));
    return AutoBuilder.followPath(path);
  }
}