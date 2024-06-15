package frc.robot.commands;

import java.util.Map;
import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.ShooterWristConstants;
import frc.robot.Constants.TeleopSwerveConstants;
import frc.robot.constants.DynamicShootingConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
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
    SwerveRequest.FieldCentric request2;

    Translation2d[] pivotPositions = new Translation2d[DynamicShootingConstants.distanceMapLength - 2];
    Translation2d[] shooterSpeeds = new Translation2d[DynamicShootingConstants.distanceMapLength - 2];

    Translation2d speakerPos;

    DoubleSupplier directionSupplier;
    Command drive;
    boolean canDriverRotate;
    int blinking = 0;

    double speakerOffset = FieldConstants.blueAllianceYOffset;
    final boolean isDebug = true;

    private enum TargetingMode {Localization, SpeakerTag, TwoDimensional};

    SendableChooser<TargetingMode> modeChooser = new SendableChooser<>();


    // This is so jank the entire class needs to be rewritten.
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
        addRequirements(shooter, pivot, swerve, leds);

        this.directionSupplier = directionSupplier;

        modeChooser.setDefaultOption("Localization", TargetingMode.Localization);
        modeChooser.addOption("Localization", TargetingMode.Localization);
        modeChooser.addOption("Speaker Tag", TargetingMode.SpeakerTag);
        //modeChooser.addOption("2D Speaker Tag", TargetingMode.TwoDimensional);

        /*pidController = new ProfiledPIDController(DynamicShootingConstants.kP, DynamicShootingConstants.kI,
                DynamicShootingConstants.kD, new Constraints(DynamicShootingConstants.kMaxAngularVelocity,
                        DynamicShootingConstants.kMaxAngularAcceleration));*/
        request = new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(6 * 0.1).withRotationalDeadband(0) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        request2 = TeleopSwerveConstants.TeleopDriveRequest;
        request.HeadingController = new PhoenixPIDController(DynamicShootingConstants.kP, DynamicShootingConstants.kI,
                DynamicShootingConstants.kD);
        request.HeadingController.setTolerance(Math.PI/128);

        
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
    if (isDebug) {
        
        SmartDashboard.putNumber("speaker Y offset", speakerOffset);
        speakerPos = new Translation2d((directionSupplier.getAsDouble() == 1) ? FieldConstants.redSpeakerX : FieldConstants.blueSpeakerX, FieldConstants.blueSpeakerY);
            
    }
    else {
        speakerPos = VisionConstants.kSpeakerPose;
    }
    // swerve.applyRequest(() -> request.withVelocityX(direction * yAxisSupplier.getAsDouble() * 4.5).withVelocityY(direction *
    // xAxisSupplier.getAsDouble() * 4.5).withTargetDirection(VisionConstants.kVisionConstants.kSpeakerPose.minus(swerve.getState().Pose.getTranslation()).getAngle()));
    
  
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
        if (isDebug && SmartDashboard.getNumber("speaker Y offset", speakerOffset) != speakerOffset)
            speakerOffset = SmartDashboard.getNumber("speaker Y offset", speakerOffset);

        if (Timer.getFPGATimestamp() - vision.lastResults[0].getTimestampSeconds() > .3 || (!vision.containsSpeakerTag(0) && modeChooser.getSelected() != TargetingMode.Localization)) {
            // Drive forward with
            swerve.setControl(request2.withVelocityX(directionSupplier.getAsDouble() * xAxisSupplier.getAsDouble() * Math.pow(Math.abs(xAxisSupplier.getAsDouble()), 3) * 6)
            .withVelocityY(directionSupplier.getAsDouble() * yAxisSupplier.getAsDouble() * Math.pow(Math.abs(yAxisSupplier.getAsDouble()), 3) * 6) // Drive left with negative X (left)
            .withRotationalRate(-rightXAxis.getAsDouble() * Math.pow(Math.abs(rightXAxis.getAsDouble()), 3) * 1.5 * Math.PI));
            return;
        }

        // negative Y (forward)
        request.withVelocityX(directionSupplier.getAsDouble() * yAxisSupplier.getAsDouble() * 4.5)
        .withVelocityY(directionSupplier.getAsDouble() * xAxisSupplier.getAsDouble() * 4.5);
        switch (modeChooser.getSelected()) {
            case Localization:
                request.withTargetDirection(
                    ((isDebug) ? speakerPos.plus(new Translation2d(0, speakerOffset)) : speakerPos).minus(swerve.getState().Pose.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180)));
                break;
            case SpeakerTag:
                request.withTargetDirection(
                    ((isDebug) ? speakerPos.plus(new Translation2d(0, speakerOffset)) : speakerPos).minus(vision.cameraPoses[VisionConstants.kShooterCameraIndex].getTranslation().toTranslation2d()).getAngle().plus(Rotation2d.fromDegrees(180)));
                break;
            default:
                request.withTargetDirection(
                    ((isDebug) ? speakerPos.plus(new Translation2d(0, speakerOffset)) : speakerPos).minus(swerve.getState().Pose.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180)));
        }
        // Find above and below keys
        swerve.setControl(request);
        double distance = VisionConstants.kSpeakerPose.plus(new Translation2d(0, speakerOffset)).minus(swerve.getState().Pose.getTranslation()).getNorm();
        Map.Entry<Double, Integer> lowEntry = DynamicShootingConstants.distanceToIndex.floorEntry(distance);
        Map.Entry<Double, Integer> highEntry = DynamicShootingConstants.distanceToIndex.ceilingEntry(distance);

        SmartDashboard.putNumber("Distance to speaker", distance);
        SmartDashboard.putNumber("Robot Angle",
                (VisionConstants.kSpeakerPose.minus(swerve.getState().Pose.getTranslation()).getAngle().getDegrees()));
        // Find and apply interpolated angle and speed
        if (lowEntry == null || lowEntry.getValue() < 0 || highEntry == null
                || highEntry.getValue() >= DynamicShootingConstants.distanceToIndex.size()) {
            leds.setToHue(1);
            SmartDashboard.putBoolean("null entry", true);
            return;
        } else {
            
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

        if (shooter.isAtReq()) {
            leds.setBlinkPattern(16, 8, 55);
        }
         
    }

    @Override
    public void end(boolean interrupted) {
        // if (CommandScheduler.getInstance().isScheduled(drive))
        //     CommandScheduler.getInstance().cancel(drive);
        shooter.stop();
        pivot.setRequest(ShooterWristConstants.kFlat);
    }

}
