package frc.robot.commands;

import java.util.Map;
import java.util.function.DoubleSupplier;

import org.opencv.core.Mat.Tuple3;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.math.spline.SplineHelper;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.PrimerConstants;
import frc.robot.Constants.ShooterWristConstants;
import frc.robot.Constants.TeleopSwerveConstants;
import frc.robot.constants.DynamicShootingConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

public class ShootAnywhereAuton extends Command {

    CommandSwerveDrivetrain swerve;
    ShooterSubsystem shooter;
    ShooterPivotSubsystem pivot;
    LEDSubsystem leds;
    PrimerSubsystem primer;
    ProfiledPIDController pidController;
    CubicHermiteSpline[] pivotSplines;
    CubicHermiteSpline[] shooterSplines;

    DoubleSupplier xAxisSupplier;
    DoubleSupplier yAxisSupplier;
    DoubleSupplier rightXAxis;

    SwerveRequest.FieldCentricFacingAngle request;
    SwerveRequest.FieldCentric request2;

    Translation2d[] pivotPositions = new Translation2d[DynamicShootingConstants.distanceMapLength - 2];
    Translation2d[] shooterSpeeds = new Translation2d[DynamicShootingConstants.distanceMapLength - 2];

    DoubleSupplier directionSupplier;
    Command drive;
    boolean canDriverRotate;

    // This is so jank the entire class needs to be rewritten;
    public ShootAnywhereAuton(CommandSwerveDrivetrain swerveSubsystem,
            ShooterSubsystem shooterSubsystem, ShooterPivotSubsystem pivotSubsystem, LEDSubsystem ledSubsystem,
            PrimerSubsystem primerSubsystem) {

        swerve = swerveSubsystem;
        shooter = shooterSubsystem;
        pivot = pivotSubsystem;
        leds = ledSubsystem;
        primer = primerSubsystem;

        addRequirements(shooter, pivot, leds, primer);

        pidController = new ProfiledPIDController(DynamicShootingConstants.kP, DynamicShootingConstants.kI,
                DynamicShootingConstants.kD, new Constraints(DynamicShootingConstants.kMaxAngularVelocity,
                        DynamicShootingConstants.kMaxAngularAcceleration));

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
        // 4.5).withTargetDirection(VisionConstants.kVisionConstants.kSpeakerPose.minus(swerve.getState().Pose.getTranslation()).getAngle()));

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

        // Find above and below keys
        double distance = VisionConstants.kSpeakerPose.minus(swerve.getState().Pose.getTranslation()).getNorm();
        Map.Entry<Double, Integer> lowEntry = DynamicShootingConstants.distanceToIndex.floorEntry(distance);
        Map.Entry<Double, Integer> highEntry = DynamicShootingConstants.distanceToIndex.ceilingEntry(distance);

        SmartDashboard.putNumber("Auton Distance to speaker", distance);
        SmartDashboard.putNumber("Auton Robot Angle",
                VisionConstants.kSpeakerPose.minus(swerve.getState().Pose.getTranslation()).getAngle().getDegrees());
        double shooterAngle;
        double shooterRPM;
        // Find and apply interpolated angle and speed
        if (lowEntry == null || lowEntry.getValue() < 0 || highEntry == null
                || highEntry.getValue() >= DynamicShootingConstants.distanceToIndex.size()) {
            leds.setToHue(1);
            SmartDashboard.putBoolean("null entry", true);
            // default values
            Tuple3<Double> values = DynamicShootingConstants.distanceMap.get(DynamicShootingConstants.distanceMapLength - 1);
            shooterRPM = values.get_1();
            shooterAngle = values.get_2();

        } else {
            leds.setToHue(65);
            SmartDashboard.putBoolean("null entry", false);
                    double interpolationValue = (distance - lowEntry.getKey()) / (highEntry.getKey() - lowEntry.getKey());

        // Will get optimised away :)
        shooterRPM = shooterSplines[lowEntry.getValue()].getPoint(interpolationValue).poseMeters.getY();
        shooterAngle = pivotSplines[lowEntry.getValue()].getPoint(interpolationValue).poseMeters.getY();
        SmartDashboard.putNumber("Shooter pivot angle", shooterAngle);
        SmartDashboard.putNumber("Shooter speed", shooterRPM);
        }

        shooter.setRequest(shooterRPM);
        pivot.setRequest(shooterAngle);
        
        if (shooter.isAtReq() && pivot.atSetpoint() && request.HeadingController.atSetpoint()) {
            primer.setSpeed(PrimerConstants.kShoot);
        }

    }

    @Override
    public void end(boolean interrupted) {
        pivot.setRequest(ShooterWristConstants.kFlat);
        primer.stop();
    }

}
