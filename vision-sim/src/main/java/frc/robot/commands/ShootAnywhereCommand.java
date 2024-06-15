package frc.robot.commands;

import java.text.FieldPosition;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Spline;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.math.spline.SplineHelper;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.constants.DynamicShootingConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

public class ShootAnywhereCommand extends Command {

    CommandSwerveDrivetrain swerve;
    Vision vision;
    ShooterSubsystem shooter;
    ShooterPivotSubsystem pivot;
    LEDSubsystem leds;
    ProfiledPIDController pidController;
    CubicHermiteSpline[] pivotSplines;
    CubicHermiteSpline[] shooterSplines;

    DoubleSupplier xAxisSupplier;
    DoubleSupplier yAxisSupplier;
    DoubleSupplier rightXAxis;

    SwerveRequest.FieldCentricFacingAngle request;

    Translation2d[] pivotPositions = new Translation2d[DynamicShootingConstants.distanceMapLength - 2];
    Translation2d[] shooterSpeeds = new Translation2d[DynamicShootingConstants.distanceMapLength - 2];

    Translation2d speakerPose;

    DoubleSupplier directionSupplier;
    Command drive;
    boolean canDriverRotate;

    // This is so jank the entire class needs to be rewritten;
    public ShootAnywhereCommand(CommandSwerveDrivetrain swerveSubsystem, Vision visionSubsystem,
            ShooterSubsystem shooterSubsystem, ShooterPivotSubsystem pivotSubsystem, LEDSubsystem ledSubsystem,
            DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier angularRequest,
            DoubleSupplier directionSupplier) {

        swerve = swerveSubsystem;
        vision = visionSubsystem;
        shooter = shooterSubsystem;
        pivot = pivotSubsystem;
        leds = ledSubsystem;
        xAxisSupplier = xAxis;
        yAxisSupplier = yAxis;
        rightXAxis = angularRequest;
        addRequirements(shooter, pivot, leds);

        this.directionSupplier = directionSupplier;

        pidController = new ProfiledPIDController(DynamicShootingConstants.kP, DynamicShootingConstants.kI,
                DynamicShootingConstants.kD, new Constraints(DynamicShootingConstants.kMaxAngularVelocity,
                        DynamicShootingConstants.kMaxAngularAcceleration));
        request = new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(6 * 0.1).withRotationalDeadband(0) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        request.HeadingController = new PhoenixPIDController(DynamicShootingConstants.kP, DynamicShootingConstants.kI,
                DynamicShootingConstants.kD);
        request.HeadingController.setTolerance(.0125);

        speakerPose = (/* DriverStation.getAlliance().get() == Alliance.Blue */true)
                ? new Translation2d(FieldConstants.blueSpeakerX, FieldConstants.blueSpeakerY)
                : new Translation2d(FieldConstants.redSpeakerX, FieldConstants.redSpeakerY);
        for (int i = 1; i < DynamicShootingConstants.distanceMapLength - 1; i++) {
            pivotPositions[i - 1] = new Translation2d(DynamicShootingConstants.distanceMap.get(i).get_0(),
                    DynamicShootingConstants.distanceMap.get(i).get_2());
            shooterSpeeds[i - 1] = new Translation2d(DynamicShootingConstants.distanceMap.get(i).get_0(),
                    DynamicShootingConstants.distanceMap.get(i).get_1());
        }
        System.out.println(shooterSpeeds);

        ControlVector[] startAndEnd = SplineHelper.getCubicControlVectorsFromWaypoints(
                new Pose2d(
                        new Translation2d(
                                DynamicShootingConstants.distanceMap.get(0).get_0(),
                                DynamicShootingConstants.distanceMap.get(0).get_2()),
                        new Rotation2d(
                                DynamicShootingConstants.distanceMap.get(1).get_0()
                                        - DynamicShootingConstants.distanceMap.get(0).get_0(),
                                DynamicShootingConstants.distanceMap.get(1).get_2()
                                        - DynamicShootingConstants.distanceMap.get(0).get_2())),
                pivotPositions,
                new Pose2d(
                        new Translation2d(
                                DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 1)
                                        .get_0(),
                                DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 1)
                                        .get_2()),
                        new Rotation2d(
                                DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 2)
                                        .get_0()
                                        - DynamicShootingConstants.distanceMap
                                                .get(DynamicShootingConstants.distanceMapLength - 1).get_0(),
                                DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 2)
                                        .get_2()
                                        - DynamicShootingConstants.distanceMap
                                                .get(DynamicShootingConstants.distanceMapLength - 1)
                                                .get_2())));

        pivotSplines = SplineHelper.getCubicSplinesFromControlVectors(startAndEnd[0], pivotPositions, startAndEnd[1]);

        startAndEnd = SplineHelper.getCubicControlVectorsFromWaypoints(
                new Pose2d(
                        new Translation2d(
                                DynamicShootingConstants.distanceMap.get(1).get_0(),
                                DynamicShootingConstants.distanceMap.get(1).get_1()),
                        new Rotation2d(
                                DynamicShootingConstants.distanceMap.get(1).get_0()
                                        - DynamicShootingConstants.distanceMap.get(0).get_0(),
                                DynamicShootingConstants.distanceMap.get(1).get_1()
                                        - DynamicShootingConstants.distanceMap.get(0).get_1())),
                shooterSpeeds,
                new Pose2d(
                        new Translation2d(
                                DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 2)
                                        .get_0(),
                                DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 2)
                                        .get_1()),
                        new Rotation2d(
                                DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 2)
                                        .get_0()
                                        - DynamicShootingConstants.distanceMap
                                                .get(DynamicShootingConstants.distanceMapLength - 1).get_0(),
                                DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 2)
                                        .get_1()
                                        - DynamicShootingConstants.distanceMap
                                                .get(DynamicShootingConstants.distanceMapLength - 1)
                                                .get_1())));

        shooterSplines = SplineHelper.getCubicSplinesFromControlVectors(startAndEnd[0], shooterSpeeds, startAndEnd[1]);
    }

  @Override
  public void initialize() {
    // swerve.applyRequest(() -> request.withVelocityX(direction *
    // yAxisSupplier.getAsDouble() * 4.5).withVelocityY(direction *
    // xAxisSupplier.getAsDouble() *
    // 4.5).withTargetDirection(speakerPose.minus(swerve.getState().Pose.getTranslation()).getAngle()));

    // ? swerve.getState().Pose.getRotation().plus(Rotation2d.fromRadians(-rightXAxis.getAsDouble() * 4.5 * .02))
    //         :
    //(Timer.getFPGATimestamp() - vision.lastResults[0].getTimestampSeconds() > .3 || canDriverRotate)
    // drive = swerve.applyRequest(() -> {
        
    //     request.withVelocityX(directionSupplier.getAsDouble() * yAxisSupplier.getAsDouble() * 4.5)
    //     .withVelocityY(directionSupplier.getAsDouble() * xAxisSupplier.getAsDouble() * 4.5);
    //     if (canDriverRotate) {
    //         return request.(-rightXAxis.getAsDouble() * Math.pow(Math.abs(rightXAxis.getAsDouble()), 3) * 1.5 * Math.PI);
    //     } else {
    //         return request.withTargetDirection(
    //          speakerPose.minus(vision.cameraPoses[0].getTranslation().toTranslation2d()).getAngle().plus(Rotation2d.fromDegrees(180)));
    //     }
    //     });
    // CommandScheduler.getInstance().schedule(drive);
  }

    public void generateValues(int count) {

        String shooterString = new String(" ");
        String pivotString = new String(" ");
        SmartDashboard.putNumber("thing", count);
        for (int i = 0; i < count; i++) {
            // Find above and below keys.
            double distance = Math.random()
                    * (-DynamicShootingConstants.distanceMap.get(0).get_0()
                            + DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 1)
                                    .get_0())
                    + DynamicShootingConstants.distanceMap.get(0).get_0();
            Map.Entry<Double, Integer> lowEntry = DynamicShootingConstants.distanceToIndex.floorEntry(distance);
            Map.Entry<Double, Integer> highEntry = DynamicShootingConstants.distanceToIndex.ceilingEntry(distance);

            // Find and apply interpolated angle and speed
            if (lowEntry == null || lowEntry.getValue() < 0 || highEntry == null
                    || highEntry.getValue() >= DynamicShootingConstants.distanceToIndex.size()) {
                // leds.setToHue(1);
                continue;
            } else {
                // leds.setToHue(65);
            }

            double interpolationValue = (distance - lowEntry.getKey()) / (highEntry.getKey() - lowEntry.getKey());

            shooterString = shooterString.concat(Double.toString(distance).concat(" ")
                    .concat(Double.toString(
                            shooterSplines[lowEntry.getValue()].getPoint(interpolationValue).poseMeters.getY()))
                    .concat("\n"));
            pivotString = pivotString.concat(Double.toString(distance) + " "
                    + Double.toString(pivotSplines[lowEntry.getValue()].getPoint(interpolationValue).poseMeters.getY())
                    + "\n");
        }
        System.out.println("Auto generated data:");
        System.out.println(shooterString);
        // System.out.println(pivotString);
        // SmartDashboard.putString("pivot", pivotString);
    }

    @Override
    public void execute() {

        // Make sure the values we are homing to are valid
        if (vision.lastResults[0].getTimestampSeconds() < 0
                || Timer.getFPGATimestamp() - vision.lastResults[0].getTimestampSeconds() > .3) {
            return;
        }
        // Find above and below keys
        double distance = speakerPose.minus(vision.cameraPoses[0].getTranslation().toTranslation2d()).getNorm();
        Map.Entry<Double, Integer> lowEntry = DynamicShootingConstants.distanceToIndex.floorEntry(distance);
        Map.Entry<Double, Integer> highEntry = DynamicShootingConstants.distanceToIndex.ceilingEntry(distance);

        SmartDashboard.putNumber("Distance to speaker", distance);
        SmartDashboard.putNumber("Robot Angle",
                speakerPose.minus(vision.cameraPoses[0].getTranslation().toTranslation2d()).getAngle().getDegrees());
        // Find and apply interpolated angle and speed
        if (lowEntry == null || lowEntry.getValue() < 0 || highEntry == null
                || highEntry.getValue() >= DynamicShootingConstants.distanceToIndex.size()) {
            leds.setToHue(1);
            SmartDashboard.putBoolean("null entry", true);
            canDriverRotate = true;
            return;
        } else {
            canDriverRotate = false;
            leds.setToHue(65);
            SmartDashboard.putBoolean("null entry", false);
        }

        double interpolationValue = (distance - lowEntry.getKey()) / (highEntry.getKey() - lowEntry.getKey());

        // Will get optimised away :)
        double shooterRPM = shooterSplines[lowEntry.getValue()].getPoint(interpolationValue).poseMeters.getY();
        double shooterAngle = pivotSplines[lowEntry.getValue()].getPoint(interpolationValue).poseMeters.getY();
        SmartDashboard.putNumber("Shooter pivot angle", shooterAngle);
        SmartDashboard.putNumber("Shooter speed", shooterRPM);

        shooter.setRequest(shooterRPM);
        pivot.setRequest(shooterAngle);

    }

    @Override
    public void end(boolean interrupted) {
        if (CommandScheduler.getInstance().isScheduled(drive))
            CommandScheduler.getInstance().cancel(drive);
    }

}
