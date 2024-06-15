package frc.robot.constants;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;

public class VisionSimConstants {
    static {
        SwerveModuleConstants = TunerConstants.createSimDrivetrain();
        SwerveDrivetrainConstants = new VisionSimConstants.Lazy<SwerveDrivetrainConstants>(
            () -> TunerConstants.SIM_DRIVETRAIN_CONSTANTS);
        initializeValues();
    }

    public static class Lazy<T> {
        private Optional<T> value = Optional.empty();
        private Supplier<T> valueSupplier;

        public Lazy(Supplier<T> valueSupplier) {
            this.valueSupplier = valueSupplier;
        }

        public T getValue() {
            if (value.isEmpty()) {
                value = Optional.of(valueSupplier.get());
            }
            return value.get();
        }
    }

    public static SwerveModuleConstants[] SwerveModuleConstants;

    public static Lazy<SwerveDrivetrainConstants> SwerveDrivetrainConstants;

    public static Lazy<SwerveModule[]> SwerveModules;
    public static Lazy<SwerveModulePosition[]> SwerveModulePositions;
    public static Lazy<Translation2d[]> SwerveModuleLocations;
    public static Lazy<SwerveDriveKinematics> Kinematics = new Lazy<SwerveDriveKinematics>(
            () -> new SwerveDriveKinematics(VisionSimConstants.SwerveModuleLocations.getValue()));
    public static final Transform3d[] kTrueCameraTransforms = {
        new Transform3d(new Translation3d(Units.inchesToMeters(-6.868435), 0, Units.inchesToMeters(12.907907)), new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180)))
    };

    public static Lazy<SimCameraProperties> CameraProperties = new Lazy<SimCameraProperties>(() -> {
        SimCameraProperties cameraProperties = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProperties.setCalibration(1200, 1600, Rotation2d.fromDegrees(94));
        // Approximate detection noise with average and standard deviation error in
        // pixels.
        cameraProperties.setCalibError(3.25, 0.25);
        // Set the camera image capture framerate (Note: this is limited by robot loop
        // rate).
        cameraProperties.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProperties.setAvgLatencyMs(35);
        cameraProperties.setLatencyStdDevMs(9);
        return cameraProperties;
    });

    private static void initializeValues() {
        SwerveModule[] modules = new SwerveModule[4];
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        Translation2d[] moduleLocations = new Translation2d[4];

        int iteration = 0;
        for (SwerveModuleConstants module : SwerveModuleConstants) {
            modules[iteration] = new SwerveModule(module,
                    VisionSimConstants.SwerveDrivetrainConstants.getValue().CANbusName);
            moduleLocations[iteration] = new Translation2d(module.LocationX, module.LocationY);
            modulePositions[iteration] = modules[iteration].getPosition(true);

            iteration++;

        }
        SwerveModules = new Lazy<SwerveModule[]>(() -> modules);
        SwerveModulePositions = new Lazy<SwerveModulePosition[]>(() -> modulePositions);
        SwerveModuleLocations = new Lazy<Translation2d[]>(() -> moduleLocations);
    }
}
