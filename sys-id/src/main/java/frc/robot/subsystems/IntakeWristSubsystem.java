package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeWristConstants;
import frc.robot.RobotContainer.currentMechanism;

public class IntakeWristSubsystem extends SubsystemBase{
    /** Motor controller for turning the intake mechanism. */
    TalonFX turningMotor;

    

    /** The required position for the turning motor. */
    double reqPosition;
    double current;
    VoltageOut voltageRequest = new VoltageOut(0);
    TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);
    boolean FOC = false;
    private SysIdRoutine m_SysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Default ramp rate is acceptable
                Volts.of((FOC) ? 10 : 4), // Reduce dynamic voltage to 4 to prevent motor brownout
                null,          // Default timeout is acceptable
                                       // Log state with Phoenix SignalLogger class
                (state)->SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts)-> turningMotor.setControl((FOC) ? currentRequest.withOutput(volts.in(Volts)) : voltageRequest.withOutput(volts.in(Volts))),
                null,
                this));

    /**
     * Creates a new IntakeSubsystem with initialized motor controllers and PID
     * controller.
     */
    public IntakeWristSubsystem(currentMechanism currMech) {

        // Initialization of motor controllers and PID controller
        turningMotor = new TalonFX(IntakeWristConstants.kIntakeTurnID);
        
        TalonFXConfiguration talonFxConfig = new TalonFXConfiguration();

        turningMotor.getConfigurator().apply(talonFxConfig);

        if (currMech == currentMechanism.intakeWrist)
            setupSysId();

    }

    private void setupSysId() {
        SignalLogger.setPath("/media/sda1/ctre-logs/");

        BaseStatusSignal.setUpdateFrequencyForAll(250,
            turningMotor.getPosition(),
            turningMotor.getVelocity(),
            turningMotor.getMotorVoltage(),
            turningMotor.getStatorCurrent(),
            turningMotor.getSupplyCurrent()
        );
        
        TalonFX.optimizeBusUtilizationForAll();

        SignalLogger.start();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.dynamic(direction);
    }

    public void testIntake() {
        turningMotor.set(0.1);
    }

    public void stopIntake() {
        turningMotor.set(0);
    }
    
    /**
     * Sets the target position for the turning motor using PID control.
     *
     * @param req The target position for the turning motor.
     */
    public void intakeToReq(double req) { 
        turningMotor.setControl(new PositionDutyCycle(req));  
    }

    /**
     * Sets the required position for the turning motor.
     *
     * @param req The required position for the turning motor.
     */
    public void setRequest(double req) {
        reqPosition = req;
    }

    /**
     * Gets the current position of the turning motor.
     *
     * @return The current position of the turning motor.
     */
    public double getPosition() {
        return turningMotor.getPosition().getValue();
    }

    public void stop() {
        turningMotor.set(0);
    }

    /**
     * Checks if the turning motor is at the required position within a specified
     * tolerance.
     *
     * @param reqPos The required position to check against.
     * @return True if the turning motor is at the required position; false
     *         otherwise.
     */
    public boolean isAtReqPosition(double reqPos) {

        if ((getPosition() >= (reqPos - IntakeWristConstants.kTolerance))
                && (getPosition() <= (reqPos + IntakeWristConstants.kTolerance))) {
            return true;
        }
        return false;
    }

    public Command indexPosCommand() {
        return runOnce(() -> intakeToReq(IntakeWristConstants.kStow)).andThen(new WaitUntilCommand(() ->  isAtReqPosition(IntakeWristConstants.kStow))).finallyDo(() -> stop());
    }
    public Command intakePosCommand() {
        return runOnce(() -> intakeToReq(IntakeWristConstants.kIntake)).andThen(new WaitUntilCommand(() ->  isAtReqPosition(IntakeWristConstants.kIntake))).finallyDo(() -> stop());
    }
    public Command stopMotorCommand(){
        return runOnce(this::stop);
    }
    /**
     * Ewww ugly get this out of here
     * @return bad
     */
    public Command intakeTest() {
        //return run(this::testIntake).finallyDo(() -> stopIntake());
        return new SequentialCommandGroup(runOnce(() -> intakeToReq(IntakeWristConstants.kIntake)).andThen(new WaitUntilCommand(() -> isAtReqPosition(IntakeWristConstants.kIntake))));
    }
  

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //System.out.println(getPosition());
        SmartDashboard.putNumber("Pose", getPosition());
        SmartDashboard.putNumber("Current", current);
        
    }
}
