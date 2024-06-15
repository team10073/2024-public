// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.opencv.core.Mat.Tuple3;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

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

  public static class IntakeRollerConstants
  {
    public static final int kIntakeRollerID = 11;
    public static final double kIntake = 0.4;
    public static final double kOuttake = -kIntake;
    public static final double kHold = 0.1;
    public static final double kStop = 0;
    public static final double kIntakeCurrentLimit = 5;
    public static final int kIntakeAnalogInputChannel = 2;// To Change
  }

  public static class IntakeWristConstants
  {
    public static final int kIntakeTurnID = 12;
    // PID constants
    public static final double kP = 0.3;
    public static final double kFF = 0.01;

    // Positions
    public static final double kIntake = 14;
    public static final double kStow = 0.0;
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
    public static final double kP = 0.0011200000118743628;
    public static final double kI = 0;
    public static final double kD = 0.0002500000118743628;
    public static final double kS = 0.17;
    public static final double kV = 0.0001654583333333333;
    
    // speed constants
    public static final double kShoot = 1/kV;//0.2 and 1.0
    public static final double kReverse = -kShoot;
    public static final double kSubwooferShot = 0.8;
    public static final double kSubwooferSpeed = 1/kV;//4640;
    public static final double kStop = 0.0;
    public static final double kPodium = 5350;

    

    public static final int kCurrentLimit = 60;
    // public static final double kFF = 0.0001654579973546788;
    public static final int kShooterAnalogInputChannel = 0;

    public static final double kTolerance = 200;

    public static final double kMotionMagicCruiseVelocity = 80;
    public static final double kMotionMagicCruiseAcceleration = 160;
    public static final double kMotionMagicJerk = 1600;
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
    public static final double kIntake = 0.3;
    public static final double kOuttake = -kIntake;
    public static final double kStop = 0.0;
    public static final double kAmp = -1.0;
    public static final double kShoot = 1.0;//1.0
    public static final double kPrimerCurrentLimit = 30;

    // PID values
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kFF = 0.0;
  }// Should go away for final competition code

  public static class LEDConstants {
    public static final int PWMPortLeft = 9;
    public static final int LEDLength = 18;
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
    public static double kPivotReduction = 60;
    public static double kOldFlat = 0.377481;
    public static double kFlat = 0.377481;
    public static double k4PPreloadShot = 0;
    public static double k4PLastShot = kFlat + Rotation2d.fromDegrees(32.4238401).getRotations();
    public static double kStartPos = 0.207 - kFlat + kOldFlat;
    public static double kPodiumPos = kFlat + 0.5 - Rotation2d.fromDegrees(21.8).getRotations();//0.801; // To change
    public static double kSubwooferPos = 0.55-kFlat+kOldFlat; // To change 0.541
    public static double kIntakePos = 0.526-kFlat+kOldFlat; // To change
    public static double kAmpPos = 0.748-kFlat + kOldFlat;// To change
    public static double kSideSubPos = Math.toRadians(67) / ( 2 * Math.PI ) + 0.372;
    public static double kTolerance = Math.toRadians(1) / ( 2 * Math.PI );
    public static double kLimit = 0.5/*5.52380952383*/ / (2 * Math.PI);
    public static double kDt = 0.02;// To change
    public static int kShooterBeambreak = 1;

    // Trapezoidal profiling
    public static double maxVelocity = 1.75; // TODO: Units and true value
    public static double maxAcceleration = 0.75; // TODO: Units and true value
    public static final double kG = 0; // Units of Volts. Calculated by voltage required to hold vertical position.
    public static final double kS = 0; // Units of Volts.
  }
}
