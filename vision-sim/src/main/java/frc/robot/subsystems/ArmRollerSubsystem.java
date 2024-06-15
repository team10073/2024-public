package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class ArmRollerSubsystem extends SubsystemBase{
    private CANSparkMax motor;
    public ArmRollerSubsystem(){
        motor = new CANSparkMax(31, MotorType.kBrushless);
    }

    public void intake(){
        motor.set(0.4);
    }
    public void outtake(){
        motor.set(-0.4);
    }
    public void stop(){
        motor.set(0.0);
    }

    public Command intakeCommand(){
        return run(this::outtake)
        .finallyDo(this::stop);

    }
}
