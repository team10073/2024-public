// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * Constants to initialize controller inputs.
   */
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeRollerConstants {
    public static final int kIntakeRollerID = 11;
    public static final double kIntake = 1.0;
    public static final double kAmp = -0.475;
    public static final double kOuttake = -0.3; // This needs to be slower than primer.
    public static final double kHold = 0.1; // Recommend .06 or .05
    public static final double kStop = 0;
    public static final double kStatorLimit = 60;
    public static final double kSupplyLimit = 40;
    public static final int kIntakeAnalogInputChannel = 2;// To Change
  }

  public static class IntakeWristConstants {
    public static final int kIntakeTurnID = 12;

    // Limits
    public static final double kStatorLimit = 60;
    public static final double kSupplyLimit = 40;

    // PID constants
    public static final double kP = 0.15;
    public static final double kFF = 0.01;

    // Positions
    public static final double kIntake = 14.438;
    public static final double kStow = 0.0;
    public static final double kAmp = 3.471008;
    public static final double kHalf = kIntake / 2;

    // tolerance
    public static final double kTolerance = 0.5;

  }

  /**
   * Constants for the IndexerSubsystem.
   * IDs and speeds are included.
   */
  public static class IndexerConstants {
    public static final int kIndexerID = 41;
    public static final double kForward = 1.0;
    public static final double kReverse = -kForward;
    public static final double kStop = 0.0;
    // Constant that sets motor inverted
    public static final boolean kMotorInvert = true;
  }

  /**
   * Constants for the ShooterSubsystem.
   * IDs, speeds, and PID values are included.
   */
  public static class ShooterConstants {

    // device IDs
    public static final int kShooterTopRollerMotorID = 21;
    public static final int kShooterBottomRollerMotorID = 22;

    // PID constants (top shooting roller)
    public static final double kP = 0.4;
    public static final double kI = 0;
    public static final double kD = 0.0;
    public static final double kS = 0.23738;
    public static final double kV = 0.11924;
    public static final double kA = 0.0054329;

    // speed constants
    public static final double kShoot = 4000;// 0.2 and 1.0
    public static final double kReverse = -kShoot;
    public static final double kSubwooferShot = 0.8;
    public static final double kSubwooferSpeed = 4500;// 4640;
    public static final double kStop = 0.0;
    public static final double kPodium = 5350;
    public static final double kFerry = 4750;
    public static final double kMidStageSpeed = 5850;
    public static final double kAngleFerry = -0.345;
    public static final double kBackpackSpeed = 5000;
    public static final double k5PUnderStageLastShotSpeed = 5000;
    public static final double k5PUnderStageFirstShotSpeed = 5000;

    // Limits
    public static final int kStatorLimit = 60;
    public static final int kSupplyLimit = 40;

    // public static final double kFF = 0.0001654579973546788;
    public static final int kShooterAnalogInputChannel = 1;

    public static final double kTolerance = 100;

    public static final double kMotionMagicCruiseVelocity = 80;
    public static final double kMotionMagicCruiseAcceleration = 160;
    public static final double kMotionMagicJerk = 1600;
    public static final double kAmpRPM = 800;
  }

  /**
   * Constants for the PrimerSubsystem.
   * IDs, speeds, and current limits are included.
   */
  public static class PrimerConstants {
    // IDs
    public static final int kPrimerRollerMotorID = 31;
    public static final int kPrimerSlotID = 0;
    // Speeds
    public static final double kIntake = 0.2;
    public static final double kOuttake = -kIntake;
    public static final double kStop = 0.0;
    public static final double kAmp = .5;
    public static final double kShoot = 1.0;// 1.0
    public static final double kStatorLimit = 80;
    public static final double kSupplyLimit = 30;
    public static final double kSlowSpeed = 0.05;
    public static final double kBackpackSpeed = 0.35;

    // PID values
    public static final double kVelocityP = 0.1;
    public static final double kVelocityI = 0.0;
    public static final double kVelocityD = 0.0;
    public static final double kVelocityS = 0.0;
    public static final double kVelocityV = 0.0;
    public static final double kVelocityA = 0.0;
    public static final double kPositionP = 10.0;
    public static final double kPositionI = 0.0;
    public static final double kPositionD = 0.05;
    public static final double kPositionS = 0.0;
    public static final double kPositionV = 0.0;
    public static final double kPositionA = 0.0;
    public static final double kFF = 0.0;
    public static final double kEncoderOffset = 0.33;
    public static final double kShotDelay = .15;
    public static final double kAmpDelay = 0.1;
  }// Should go away for final competition code

  public static class LEDConstants {
    public static final int PWMPortLeft = 0;
    public static final int LEDLength = 14;
    public static final int coneHValue = 18;
    public static final int coneSValue = 255;
    public static final int coneVValue = 130;
    public static final int cubeHValue = 134;
    public static final int cubeSValue = 255;
    public static final int cubeVValue = 130;
  }

  public static class ShooterWristConstants {
    public static int kShooterMasterID = 52;
    public static int kShooterSlaveID = 51;

    // Limits
    public static final double kStatorLimit = 60;
    public static final double kSupplyLimit = 40;

    public static double kPivotReduction = 60;
    public static double kFlat = -0.213379;
    public static double kSubwooferPos = -0.389404; // To change 0.541
    public static double kIntakePos = -0.348389; // To change
    public static double kBackPackPos = -0.580322;
    public static double kAmpSetupPosition = -0.390381;
    public static double kFerry = kIntakePos;
    public static double kMidStagePos = -.348389;
    public static double k5PUnderStageLastShotPos = kFlat - Units.degreesToRotations(33.568);
    public static double kTolerance = Math.toRadians(1) / (2 * Math.PI); // Just divide by 360.0???
    public static double kLimit = 0.5/* 5.52380952383 */ / (2 * Math.PI);
    public static double kDt = 0.02;// To change
    public static int kShooterBeambreak = 1;

    // Trapezoidal profiling
    public static double maxVelocity = 1.75; // TODO: Units and true value
    public static double maxAcceleration = 0.75; // TODO: Units and true value
    public static final double kG = 0; // Units of Volts. Calculated by voltage required to hold vertical position.
    public static final double kS = 0; // Units of Volts.
    public static final double kP = 12 * 6.4;
    public static final double kD = 0.0;
    public static final double kStowpos = kFlat;
    public static final double kOuttake = -0.1;
    public static final double kAmpFlickPosition = -.05;
    public static final double k5PUnderStageFirstShotPos = -.33;
  }

  public static class BackpackWristConstants {
    public static final int kMotorID = 62;

    // Limits
    public static final double kStatorLimit = 40;
    public static final double kSupplyLimit = 30;
    public static final double kMaxForwardDutyCycle = .15;
    public static final double kMaxReverseDutyCycle = .05;

    // Wrist Characteristics
    public static final double kGearRatio = 275.0 / 9.0;
    public static final double kRangeDegrees = 192;

    // PID constants
    public static final double kP = 2.0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;

    // Positions
    public static final double kIntake = 15.0;//Units.degreesToRotations(192) * kGearRatio;
    public static final double kStow = 0;

    // tolerance
    public static final double kTolerance = Units.degreesToRotations(2);

  }
  public static class BackpackRollerConstants {
    public static final int kBackpackRollerID = 61;
    public static final double kIntake = 0.7;
    public static final double kOuttake = -.7;
    public static final double kHold = 0.1;
    public static final double kStop = 0;
    public static final double kStatorLimit = 60;
    public static final double kSupplyLimit = 30;
    
  }

  public static class TeleopSwerveConstants {
    public static double MaxSpeedMetersPerSec = 6; // 6 meters per second desired top speed
    public static double MaxAngularRateRotPerSec = 2.25 * Math.PI; // 6/4 of a rotation per second max angular velocity
    public static double SwerveMagnitudeExponent = 4;
    public static Rotation2d kBackpackAlignAngle = Rotation2d.fromDegrees(180-37);
    public static Rotation2d kFerryAlignAngle = Rotation2d.fromDegrees(-37);

    public static final SwerveRequest.FieldCentric TeleopDriveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeedMetersPerSec * Math.pow(.1, 4))
        .withRotationalDeadband(MaxAngularRateRotPerSec * Math.pow(.1, 4)) // Add a 10% deadband.
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
    public static final double kHeadingTolerance = Units.degreesToRadians(.5);
  }
}
