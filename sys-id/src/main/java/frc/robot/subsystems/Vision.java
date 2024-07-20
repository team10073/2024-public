package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.constants.VisionConstants;

public class Vision {
    Thread visionThread;
    PhotonCamera[] cameras = new PhotonCamera[VisionConstants.kCameraCount];
    public PhotonPipelineResult[] lastResults = new PhotonPipelineResult[VisionConstants.kCameraCount];
    public Pose3d[] cameraPoses = new Pose3d[VisionConstants.kCameraCount];
    Pose3d robotPose; // Might use this in other filter methods later
    AprilTagFieldLayout field;
    Pose3d tempPose;
    CommandSwerveDrivetrain swerve;
    boolean continueLoop;
    int speakerID;


    public Vision(CommandSwerveDrivetrain swerveDrive) {

        for (int i = 0; i < VisionConstants.kCameraCount; i++) {
            cameras[i] = new PhotonCamera(VisionConstants.kCameraNames[i]);
        }

        getResult();

        swerve = swerveDrive;

        try {
            field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        }
        catch (Exception e) {
            SmartDashboard.putString("Vision Error Message", e.getMessage());
        }

        visionThread = new Thread(() -> {
            while (true) {
                getResult();
                
                filter3();
            }
        });

        if (DriverStation.getAlliance().isPresent())
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                speakerID = 7;
            }
            else {
                speakerID = 4;
            }

        visionThread.setName("Vision Thread");

        visionThread.start();
    }

    public void stopThread() {
        try {
        visionThread.join();
        } catch (Exception e) {

        }
    }

    public void startThread() {
        visionThread.start();
    }

    private void getResult() {
        for (int i = 0; i < VisionConstants.kCameraCount; i++) {
            lastResults[i] = cameras[i].getLatestResult();
        }
    }

    private Pose3d bestTargetToRobotPose(PhotonTrackedTarget target, int cameraNumber) {
        return field.getTagPose(target.getFiducialId()).get()
            .transformBy(target.getBestCameraToTarget().inverse())
            .transformBy(VisionConstants.kCameraTransforms[cameraNumber].inverse());
    }

    private Pose3d alternateTargetToRobotPose(PhotonTrackedTarget target, int cameraNumber) {
        return field.getTagPose(target.getFiducialId()).get()
            .transformBy(target.getBestCameraToTarget().inverse())
            .transformBy(VisionConstants.kCameraTransforms[cameraNumber].inverse());
    }

    /**
     * Filtering methods used:
     * - ID: The tags with IDs not on the field are discarded.
     * - Distance: Distant tags are discarded due to the degrading quality of the tag.
     * - <EXPERIMENTAL> Angular: The robot angle from and tag and gyro are compared.
     *   If the vision angle is out of tolerance it is discarded.
     * - <EXPERIMENTAL> Perpendicularity: If the angle between the the camera plane and
     *   the tag plane is too small to filter out with the angular filter.
     */
    public void filter1() {

        for (int i = 0; i < VisionConstants.kCameraCount; i++) {

            SmartDashboard.putNumber("getTimestampSeconds", lastResults[i].getTimestampSeconds());
            SmartDashboard.putNumber("FPGA Timestamp - latency", Timer.getFPGATimestamp() - lastResults[i].getLatencyMillis() / 1000.0);

            for (PhotonTrackedTarget target : lastResults[i].targets) {

                if (target.getFiducialId() > 16 || target.getFiducialId() < 1)
                    continue;
                
                // Double check this math and method later
                /* According to my current theory on tag flipping being kind of like a mirror over the line y=x 
                 * either both or neither of the tags will be withing our angle margin so we don't have to test the actual poses.
                 */
                if (Math.abs(field.getTagPose(target.getFiducialId()).get().getRotation().getZ() - (swerve.getRotation3d().getZ() + VisionConstants.kCameraTransforms[i].getZ())) < VisionConstants.kAngularPerpendicularityCutoff)
                    continue;
                
                tempPose = bestTargetToRobotPose(target, i);

                if (Math.abs(tempPose.getRotation().getZ() - swerve.getRotation3d().getZ()) < VisionConstants.kAngleMargin
                    && target.getBestCameraToTarget().getTranslation().getNorm() < VisionConstants.kDistanceCutoff) {

                    SmartDashboard.putString(VisionConstants.kCameraNames[i] + " pose", tempPose.toString());

                    // This must be here in order to try until the swerve drive unlocks the pose estimator.
                    do {
                        // Rohan wouldn't let me use for loop :(
                        continueLoop = false;
                        try {
                            swerve.addVisionMeasurement(tempPose.toPose2d(), lastResults[i].getTimestampSeconds());
                        } catch (Exception e) {
                            continueLoop = true;
                        }
                    } while (continueLoop == true);
                    
                    continue;
                }
                
                tempPose = alternateTargetToRobotPose(target, i);
                
                if (Math.abs(tempPose.getRotation().getZ() - swerve.getRotation3d().getZ()) < VisionConstants.kAngleMargin
                    && target.getAlternateCameraToTarget().getTranslation().getNorm() < VisionConstants.kDistanceCutoff) {
                    
                    SmartDashboard.putString(VisionConstants.kCameraNames[i] + " pose", tempPose.toString());

                    // This must be here in order to try until the swerve drive unlocks the pose estimator.
                    do {
                        continueLoop = false;
                        try {
                            swerve.addVisionMeasurement(tempPose.toPose2d(), lastResults[i].getTimestampSeconds());
                        } catch (Exception e) {
                            continueLoop = true; // This could all be fixed with a goto...
                        }
                    } while (continueLoop == true);
                }
            }
        }
    }

    /**
     * Filter methods used:
     * - ID: If the Fiducial ID isn't used on the field the target is discarded.
     * - Distance: Distant tags are discarded due to degradation of the tag resolution.
     * - <EXPERIMENTAL> Elevation: The tag transform closest to the ground is chosen.
     *   Pros: Works regardless of angle relative to the tag.
     *   Cons: Ineffective if the tag and the camera have similar elevations.
     * - <EXPERIMENTAL> Angular: The robot angle from and tag and gyro are compared.
     *   If the vision angle is out of tolerance it is discarded.
     */
    private void filter2() {
        for (int i = 0; i < VisionConstants.kCameraCount; i++) {

            SmartDashboard.putNumber("getTimestampSeconds", lastResults[i].getTimestampSeconds());
            SmartDashboard.putNumber("FPGA Timestamp - latency", Timer.getFPGATimestamp() - lastResults[i].getLatencyMillis() / 1000.0);

            for (PhotonTrackedTarget target : lastResults[i].targets) {

                if (target.getFiducialId() > 16 || target.getFiducialId() < 1)
                    continue;
                
                // Transforms to the pose of the camera, not the robot.
                tempPose = field.getTagPose(target.getFiducialId()).get()
                    .transformBy(target.getBestCameraToTarget().inverse());
                
                /*
                 * The Math.abs on the raw z position is only necessary if we don't know whether we are above or below the AprilTag.
                 * Assuming I have written this correctly, the Z component of the best pose from the camera perspectective should be
                 * reflected accross the plane where the Z is equal to the tag height. Then the distance from 0 is compared and
                 * depending on which is smaller the best or alternate tag transform is chosen.
                 */
                if (Math.abs(tempPose.getZ()) < Math.abs(-(tempPose.getZ() - field.getTagPose(target.getFiducialId()).get().getZ()) + field.getTagPose(target.getFiducialId()).get().getZ())
                    && Math.abs(tempPose.getRotation().getZ() - swerve.getRotation3d().getZ()) < VisionConstants.kAngleMargin
                    && target.getBestCameraToTarget().getTranslation().getNorm() < VisionConstants.kDistanceCutoff) {

                    // Transforms from camera to robot pose.
                    tempPose = tempPose.transformBy(VisionConstants.kCameraTransforms[i].inverse());

                    SmartDashboard.putString(VisionConstants.kCameraNames[i] + " pose", tempPose.toString());

                    // This must be here in order to try until the swerve drive unlocks the pose estimator.
                    do {
                        // Rohan wouldn't let me use for loop :(
                        continueLoop = false;
                        try {
                            swerve.addVisionMeasurement(tempPose.toPose2d(), lastResults[i].getTimestampSeconds());
                        } catch (Exception e) {
                            continueLoop = true;
                        }
                    } while (continueLoop == true);
                    
                    continue;
                }
                
                tempPose = alternateTargetToRobotPose(target, i);

                // I should check if the normal is the same on both flipped and unflipped tags.
                // Could decrease the verbosity of the function quite a bit.
                if (Math.abs(tempPose.getRotation().getZ() - swerve.getRotation3d().getZ()) < VisionConstants.kAngleMargin
                    && target.getAlternateCameraToTarget().getTranslation().getNorm() < VisionConstants.kDistanceCutoff) {

                    SmartDashboard.putString(VisionConstants.kCameraNames[i] + " pose", tempPose.toString());

                    // This must be here in order to try until the swerve drive unlocks the pose estimator.
                    do {
                        continueLoop = false;
                        try {
                            swerve.addVisionMeasurement(tempPose.toPose2d(), lastResults[i].getTimestampSeconds());
                        } catch (Exception e) {
                            continueLoop = true; // This could all be fixed with a goto...
                        }
                    } while (continueLoop == true);
                }
            }
        }
    }

    /**
     * Filter methods used:
     * - ID: If the Fiducial ID isn't used on the field the target is discarded.
     * - Distance: Distant tags are discarded due to degradation of the tag resolution.
     * - <EXPERIMENTAL> Elevation: The tag transform closest to the ground is chosen.
     *   Pros: Works regardless of angle relative to the tag.
     *   Cons: Ineffective if the tag and the camera have similar elevations.
     */
    private void filter3() {
        for (int i = 0; i < VisionConstants.kCameraCount; i++) {

            SmartDashboard.putNumber("getTimestampSeconds" + " " + Integer.toString(i), lastResults[i].getTimestampSeconds());
            SmartDashboard.putNumber("FPGA Timestamp - latency" + " " + Integer.toString(i), Timer.getFPGATimestamp() - lastResults[i].getTimestampSeconds() - lastResults[i].getLatencyMillis() / 1000.0);

            for (PhotonTrackedTarget target : lastResults[i].targets) {

                if (target.getFiducialId() > 16 || target.getFiducialId() < 1)
                    continue;
                
                // Transforms to the pose of the camera, not the robot.
                tempPose = field.getTagPose(target.getFiducialId()).get()
                    .transformBy(target.getBestCameraToTarget().inverse());
                
                /*
                 * The Math.abs on the raw z position is only necessary if we don't know whether we are above or below the AprilTag.
                 * Assuming I have written this correctly, the Z component of the best pose from the camera perspectective should be
                 * reflected accross the plane where the Z is equal to the tag height. Then the distance from 0 is compared and
                 * depending on which is smaller the best or alternate tag transform is chosen.
                 */
                if (Math.abs(tempPose.getZ()) < Math.abs(-(tempPose.getZ() - field.getTagPose(target.getFiducialId()).get().getZ()) + field.getTagPose(target.getFiducialId()).get().getZ())
                    && target.getBestCameraToTarget().getTranslation().getNorm() < VisionConstants.kDistanceCutoff) {
                    
                    // Transforms from camera to robot pose.
                    tempPose = tempPose.transformBy(VisionConstants.kCameraTransforms[i].inverse());

                    SmartDashboard.putString(VisionConstants.kCameraNames[i] + " pose", tempPose.toString());
                    
                    if (target.getFiducialId() == speakerID) {
                        cameraPoses[i] = tempPose;
                    }
                    
                    // This must be here in order to try until the swerve drive unlocks the pose estimator.
                    do {
                        // Rohan wouldn't let me use for loop :(
                        continueLoop = false;
                        try {
                            
                            //swerve.addVisionMeasurement(tempPose.toPose2d(), lastResults[i].getTimestampSeconds());
                        } catch (Exception e) {
                            continueLoop = true;
                        }
                    } while (continueLoop == true);
                    
                    continue;
                }
                
                tempPose = alternateTargetToRobotPose(target, i);

                // I should check if the normal is the same on both flipped and unflipped tags.
                // Could decrease the verbosity of the function quite a bit.
                if (Math.abs(tempPose.getRotation().getZ() - swerve.getRotation3d().getZ()) < VisionConstants.kAngleMargin
                    && target.getAlternateCameraToTarget().getTranslation().getNorm() < VisionConstants.kDistanceCutoff) {
                    
                    SmartDashboard.putString(VisionConstants.kCameraNames[i] + " pose", tempPose.toString());
                    
                    if (target.getFiducialId() == speakerID) {
                        cameraPoses[i] = tempPose;
                    }
                    
                    // This must be here in order to try until the swerve drive unlocks the pose estimator.
                    do {
                        continueLoop = false;
                        try {
                            //cameraPoses[i] = tempPose;
                            //swerve.addVisionMeasurement(tempPose.toPose2d(), lastResults[i].getTimestampSeconds());
                        } catch (Exception e) {
                            continueLoop = true; // This could all be fixed with a goto...
                        }
                    } while (continueLoop == true);
                }
            }
        }
    }

    /**
     * Was it really worth all that effort?
     * Filter methods used:
     * - ID: If the Fiducial ID isn't used on the field the target is discarded.
     * - Distance: Distant tags are discarded due to degradation of the tag resolution.
     * - Ambiguity: Tags with greater that 0.2 pose ambiguity are rejected.
     *   The best pose transform is always used. No alternates are considered.
     */
    private void filter4() {
        for (int i = 0; i < VisionConstants.kCameraCount; i++) {

            SmartDashboard.putNumber("getTimestampSeconds", lastResults[i].getTimestampSeconds());
            SmartDashboard.putNumber("FPGA Timestamp - latency", Timer.getFPGATimestamp() - lastResults[i].getLatencyMillis() / 1000.0);

            for (PhotonTrackedTarget target : lastResults[i].targets) {

                if (target.getFiducialId() > 16 || target.getFiducialId() < 1 || target.getPoseAmbiguity() > VisionConstants.kAmbiguityCutoff)
                    continue;
                
                // Transforms to the pose of the camera, not the robot.
                tempPose = bestTargetToRobotPose(target, i);
                
                /*
                 * The Math.abs on the raw z position is only necessary if we don't know whether we are above or below the AprilTag.
                 * Assuming I have written this correctly, the Z component of the best pose from the camera perspectective should be
                 * reflected accross the plane where the Z is equal to the tag height. Then the distance from 0 is compared and
                 * depending on which is smaller the best or alternate tag transform is chosen.
                 */
                if (tempPose.getTranslation().getNorm() < VisionConstants.kDistanceCutoff) {
                
                    SmartDashboard.putString(VisionConstants.kCameraNames[i] + " pose", tempPose.toString());

                    // This must be here in order to try until the swerve drive unlocks the pose estimator.
                    do {
                        // Rohan wouldn't let me use for loop :(
                        continueLoop = false;
                        try {
                            swerve.addVisionMeasurement(tempPose.toPose2d(), lastResults[i].getTimestampSeconds());
                        } catch (Exception e) {
                            continueLoop = true;
                        }
                    } while (continueLoop == true);
                }
            }
        }
    }
}
