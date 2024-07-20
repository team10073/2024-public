package frc.robot;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SysIdSwerveTranslation;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
//import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.TunerConstants;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    Alliance alliance;
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    boolean enableSignalLogging = false;
    private final VoltageOut m_sysidControl = new VoltageOut(0);
    SwerveRequest.SysIdSwerveSteerGains sysidSteerRequest = new SwerveRequest.SysIdSwerveSteerGains();
    SwerveRequest.SysIdSwerveTranslation sysidTranslationRequest = new SwerveRequest.SysIdSwerveTranslation();
    SwerveRequest.SysIdSwerveRotation sysidRotationRequest = new SwerveRequest.SysIdSwerveRotation();

    private SysIdRoutine steerRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Default ramp rate is acceptable
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                null,          // Default timeout is acceptable
                                       // Log state with Phoenix SignalLogger class
                (state)->SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts)-> sysidSteerRequest.withVolts(volts).apply(m_requestParameters, Modules),
                null,
                this));
    private SysIdRoutine translationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Default ramp rate is acceptable
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                null,          // Default timeout is acceptable
                                       // Log state with Phoenix SignalLogger class
                (state)->SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts)-> sysidTranslationRequest.withVolts(volts).apply(m_requestParameters, Modules),
                null,
                this));
    private SysIdRoutine rotationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Default ramp rate is acceptable
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                null,          // Default timeout is acceptable
                                       // Log state with Phoenix SignalLogger class
                (state)->SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts)-> sysidRotationRequest.withVolts(volts).apply(m_requestParameters, Modules),
                null,
                this));

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        setVisionMeasurementStdDevs(VisionConstants.kVisionStdDeviations);

        configurePathPlanner();

        if (enableSignalLogging)
            configSignalLogger();
        
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        setVisionMeasurementStdDevs(VisionConstants.kVisionStdDeviations);

        configurePathPlanner();

        for (SwerveModule module : Modules) {
            module.getSteerMotor();
        }

        if (enableSignalLogging)
            configSignalLogger();
        
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        
        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(2.8, 0, 0),
                                            new PIDConstants( 2.8, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            ()->{
                    if (DriverStation.getAlliance().isPresent())
                        return DriverStation.getAlliance().get() == Alliance.Red;
                    else
                        return false;
                }, // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
        for (SwerveModule module : Modules) {
            module.getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }
         
    private void configSignalLogger() {
        SignalLogger.setPath("/media/sda1/ctre-logs/");

        BaseStatusSignal.setUpdateFrequencyForAll(
        250,
        Modules[0].getDriveMotor().getMotorVoltage(),
        Modules[0].getDriveMotor().getPosition(),
        Modules[0].getDriveMotor().getVelocity(),
        Modules[0].getSteerMotor().getMotorVoltage(),
        Modules[0].getSteerMotor().getPosition(),
        Modules[0].getSteerMotor().getVelocity(),
        Modules[1].getDriveMotor().getMotorVoltage(),
        Modules[1].getDriveMotor().getPosition(),
        Modules[1].getDriveMotor().getVelocity(),
        Modules[1].getSteerMotor().getMotorVoltage(),
        Modules[1].getSteerMotor().getPosition(),
        Modules[1].getSteerMotor().getVelocity(),
        Modules[2].getDriveMotor().getMotorVoltage(),
        Modules[2].getDriveMotor().getPosition(),
        Modules[2].getDriveMotor().getVelocity(),
        Modules[2].getSteerMotor().getMotorVoltage(),
        Modules[2].getSteerMotor().getPosition(),
        Modules[2].getSteerMotor().getVelocity(),
        Modules[3].getDriveMotor().getMotorVoltage(),
        Modules[3].getDriveMotor().getPosition(),
        Modules[3].getDriveMotor().getVelocity(),
        Modules[3].getSteerMotor().getMotorVoltage(),
        Modules[3].getSteerMotor().getPosition(),
        Modules[3].getSteerMotor().getVelocity());

        // Might be able to use optimize all but kinda time crunch rn
        for (SwerveModule module : Modules) {
            module.getSteerMotor().optimizeBusUtilization();
            module.getDriveMotor().optimizeBusUtilization();
        }

        SignalLogger.start();
         
        
    }

    public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
        return steerRoutine.quasistatic(direction);
    }

    public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
        return steerRoutine.dynamic(direction);
    }

    public Command sysIdTranslationQuasistatic(SysIdRoutine.Direction direction) {
        return translationRoutine.quasistatic(direction);
    }

    public Command sysIdTranslationDynamic(SysIdRoutine.Direction direction) {
        return translationRoutine.dynamic(direction);
    }

    public Command sysIdRotationQuasistatic(SysIdRoutine.Direction direction) {
        return rotationRoutine.quasistatic(direction);
    }

    public Command sysIdRotationDynamic(SysIdRoutine.Direction direction) {
        return rotationRoutine.dynamic(direction);
    }


    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}