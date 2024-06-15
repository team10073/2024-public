package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.BackpackWristConstants;

public class BackpackWristSubsystem extends SubsystemBase{
    
    TalonFX wrist;
    double target;

    boolean isDebug = true;
    
    public BackpackWristSubsystem() {

        wrist = new TalonFX(BackpackWristConstants.kMotorID);
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        Slot0Configs pidController = configs.Slot0;
        pidController.kP = BackpackWristConstants.kP;
        pidController.kD = BackpackWristConstants.kD;
        
        configs.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(BackpackWristConstants.kStatorLimit)
            .withSupplyCurrentLimit(BackpackWristConstants.kSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);

        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.PeakForwardDutyCycle = BackpackWristConstants.kMaxForwardDutyCycle;
        configs.MotorOutput.PeakReverseDutyCycle = BackpackWristConstants.kMaxReverseDutyCycle;
        
        wrist.getConfigurator().apply(configs);

        wrist.setPosition(0);

        if (isDebug)
            configureDebug();
    }

    public void setRequest(double targetPos) {
        target = targetPos;
        wrist.setControl(new PositionVoltage(target));
    }

    private void configureDebug() {
        SmartDashboard.putNumber("Backpack target position", target);
    }



    public boolean isAtReq(double targetPos) {
        return Math.abs(targetPos - wrist.getPosition().getValueAsDouble()) < BackpackWristConstants.kTolerance;
    }

     public boolean isAtSetpoint() {
        return Math.abs(target - wrist.getPosition().getValueAsDouble()) < BackpackWristConstants.kTolerance;
    }

    public Command setRequestCommand(double targetPos) {
        return runOnce(() -> setRequest(targetPos)).andThen(new WaitUntilCommand(() -> isAtReq(targetPos)));
    }

    public Command goToBackPackPos() {
        return setRequestCommand(BackpackWristConstants.kIntake);
    }

    @Override
    public void periodic() {
        // if (isDebug && SmartDashboard.getNumber("Backpack target position", target) != target) {
        //     setRequest(SmartDashboard.getNumber("Backpack target position", target));
        // }

        SmartDashboard.putNumber("Backpack wrist position", wrist.getPosition().getValueAsDouble());


    }
    
}
