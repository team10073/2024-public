package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BackpackRollerConstants;

public class BackpackRollerSubsystem extends SubsystemBase{

    CANSparkMax roller;
    boolean isDebug = true;
    double rollerSpeed = 0;

    public BackpackRollerSubsystem() {
        roller = new CANSparkMax(BackpackRollerConstants.kBackpackRollerID, MotorType.kBrushless);
        roller.setIdleMode(IdleMode.kCoast);

        if (isDebug) {
            configureDebug();
        }
    }

    void configureDebug() {
        SmartDashboard.putNumber("Backpack roller speed", rollerSpeed);
    }

    public void setRollerSpeed(double speed) {
        roller.set(speed);
    }

    public Command setAmpSpeedCommand() { //bad naming lol
        return runOnce(() -> setRollerSpeed(BackpackRollerConstants.kOuttake));
    }

    public void stop() {
        roller.set(0);
    }

    @Override
    public void periodic() {
        // if (isDebug && SmartDashboard.getNumber("Backpack roller speed", rollerSpeed) != rollerSpeed) {
        //     rollerSpeed = SmartDashboard.getNumber("Backpack roller speed", rollerSpeed);
        //     roller.set(rollerSpeed);
        // }
    }
}
