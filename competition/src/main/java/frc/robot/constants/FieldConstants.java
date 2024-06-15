package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final double blueAllianceYOffset = 
  18.00;
  public static final double fieldLength = Units.inchesToMeters(651.223);
  public static final double blueSpeakerX = Units.inchesToMeters(18.055 / 2.0);
  public static final double blueSpeakerY = Units.inchesToMeters(218.29 + blueAllianceYOffset);
  public static final double redSpeakerX = fieldLength - blueSpeakerX;
  public static final double redSpeakerY = Units.inchesToMeters(218.29);
}
