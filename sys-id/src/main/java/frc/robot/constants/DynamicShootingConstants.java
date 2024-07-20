package frc.robot.constants;

import java.util.ArrayList;
import java.util.TreeMap;

import org.opencv.core.Mat.Tuple3;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DynamicShootingConstants {

  public static final double kP = 2.8;
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

    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(30 + 16.25), 4500.0, .553)); // Tested is .55 but if sub is good then this must be higher bc no monotonicity.
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(37 + 16.25), 4500.0,.55)); // Test this one
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(48 + 16.25), 4500.0, .529));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(59 + 16.25), 4500.0, .521));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(37+32 + 16.25), 4500.0, .507));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(37+44 + 16.25), 4500.0, .497));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(37+54 + 16.25), 4750.0, .492));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(37+64 + 16.25), 4875.0, .485));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(37+74 + 16.25), 5000.0, .479));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(37+83 + 16.25), 5250.0, .474));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(37+92 + 16.25), 5250.0, .472));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(37+107 + 16.25), 5250.0, .469));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(37+121 + 16.25), 5250.0, .4655));
    tempMap.add(new Tuple3<Double>(Units.inchesToMeters(37+134 + 16.25), 5250.0, .465));

    distanceMap = tempMap;
    distanceMapLength = tempMap.size();
    distanceToIndex = new TreeMap<>();

    for (int i = 0; i < distanceMap.size(); i++)
      distanceToIndex.put(distanceMap.get(i).get_0(), i);
    
  };
}
