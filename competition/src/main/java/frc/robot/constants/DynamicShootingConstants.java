package frc.robot.constants;

import java.util.ArrayList;
import java.util.TreeMap;

import org.opencv.core.Mat.Tuple3;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DynamicShootingConstants {

  public static final double kP = 15;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kMaxAngularVelocity = 4; // Radians per second
  public static final double kMaxAngularAcceleration = 4; // Radians per second squared

  public static final Translation2d blueSpeakerCoordinates = new Translation2d(0, 3);
  public static final Translation2d redSpeakerCoordinates = new Translation2d(17, 3);
  
  public static final double mapStepSize = 0.3;
  public static final ArrayList<Tuple3<Double>> distanceMap;
  public static final TreeMap<Double, Integer> distanceToIndex;
  public static final int distanceMapLength;

  static {
    
    ArrayList<Tuple3<Double>> tempMap = new ArrayList<Tuple3<Double>>();

    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 25 + 16.25), 5150.0, -0.394)); // Tested is .55 but if sub is good then this must be higher bc no monotonicity.
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 37 + 16.25), 5150.0, -0.3683));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 48 + 16.25), 5150.0,-0.366)); // Test this one
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 59 + 16.25), 5150.0, -0.346));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 70 + 16.25), 5200.0, -0.338));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 81 + 16.25), 5250.0, -0.327));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 92 + 16.25), 5300.0, -0.318));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 103 + 16.25), 5350.0, -0.315));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 114 + 16.25), 5450.0, -0.308));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 125 + 16.25), 5550.0, -0.302));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 136 + 16.25), 5650.0, -0.299));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 147 + 16.25), 5700.0, -0.298));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 158 + 16.25), 5800.0, -0.296));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 169 + 16.25), 5900.0, -0.295));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 181 + 16.25), 6100.0, -0.2935));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(-FieldConstants.blueSpeakerX + 192 + 16.25), 6100.0, -0.2932));
    distanceMap = tempMap;
    distanceMapLength = tempMap.size();
    distanceToIndex = new TreeMap<>();

    for (int i = 1; i < distanceMap.size() - 1; i++)
      distanceToIndex.put(distanceMap.get(i).get_0(), i);
    
  };
}
