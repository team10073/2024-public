package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PrimerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class handlePrimerShooter extends Command{
    public BooleanSupplier isAmp;
    private PrimerSubsystem primer;
    private ShooterSubsystem shooter;
    public handlePrimerShooter(PrimerSubsystem primer, BooleanSupplier isAmp){
        this.primer = primer;
        this.isAmp = isAmp;
        addRequirements(primer);
    }
    @Override
    public void execute() {
        if (isAmp.getAsBoolean()) {
            primer.setSpeed(PrimerConstants.kAmp);
        } else {   
            primer.setSpeed(PrimerConstants.kShoot);
        }
    }
    @Override 
    public void end(boolean interrupted) {
        primer.setSpeed(0);
    }
    
}
