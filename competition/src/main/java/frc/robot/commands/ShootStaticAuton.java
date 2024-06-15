package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PrimerConstants;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootStaticAuton extends Command {
    private ShooterPivotSubsystem shooterPivotSubsystem;
    private PrimerSubsystem primerSubsystem;
    private double pivotAngle;

    public ShootStaticAuton(ShooterPivotSubsystem shooterPivotSubsystem, PrimerSubsystem primerSubsystem, double pivotAngle) {
        this.shooterPivotSubsystem = shooterPivotSubsystem;
        this.primerSubsystem = primerSubsystem;
        this.pivotAngle = pivotAngle;
    }

    @Override
    public void initialize() {
        shooterPivotSubsystem.setRequest(pivotAngle);
        
    } 

    @Override
    public void execute() {
        if (shooterPivotSubsystem.atSetpoint()) {
            primerSubsystem.setSpeed(PrimerConstants.kShoot);
        }
    }
}
