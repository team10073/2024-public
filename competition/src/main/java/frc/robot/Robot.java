// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat.Tuple3;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.DynamicShootingConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    //m_robotContainer.vision.startThread();

    SmartDashboard.putData(m_robotContainer.autoChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if (m_robotContainer.autoChooser.getSelected() != m_robotContainer.currentAuton) {
      if (m_robotContainer.autoChooser.getSelected().startsWith("!")) {
        m_robotContainer.currentAuton = m_robotContainer.autoChooser.getSelected();
        m_robotContainer.autonCommand = NamedCommands.getCommand(m_robotContainer.currentAuton.substring(1));
        return;
      }
        
      m_robotContainer.autonCommand = m_robotContainer.drivetrain
          .getAutoPath(m_robotContainer.autoChooser.getSelected());
      m_robotContainer.currentAuton = m_robotContainer.autoChooser.getSelected();
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    setVisionConstants();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    setVisionConstants();
    m_robotContainer.setTeleopInitState();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
    Tuple3<Double> values = DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 1);
    double shooterRPM = values.get_1();
    double shooterAngle = values.get_2();
    System.out.println("RPM: " + shooterRPM);
    System.out.println("Angle " + shooterAngle);
  }

  private void setVisionConstants() {
    if (!DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() == Alliance.Blue) {
      m_robotContainer.temp = -1;
      VisionConstants.kSpeakerId = VisionConstants.kBlueSpeakerCenterId;
      VisionConstants.kSpeakerPose = new Translation2d(FieldConstants.blueSpeakerX, FieldConstants.blueSpeakerY);
    } else {
      m_robotContainer.temp = 1;
      VisionConstants.kSpeakerId = VisionConstants.kRedSpeakerCenterId;
      VisionConstants.kSpeakerPose = new Translation2d(FieldConstants.redSpeakerX, FieldConstants.redSpeakerY);
    }
  }

}
