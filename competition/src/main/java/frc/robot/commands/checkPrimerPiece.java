package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.*;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.PrimerSubsystem;

public class checkPrimerPiece extends Command {
    private PrimerSubsystem m_PrimerSubsystem;
    private IntakeRollerSubsystem m_IntakeRollerSubsystem;

    public checkPrimerPiece(PrimerSubsystem primerSubsystem, IntakeRollerSubsystem intakeRollerSubsystem) {
        m_PrimerSubsystem = primerSubsystem;
        m_IntakeRollerSubsystem = intakeRollerSubsystem;
        addRequirements(primerSubsystem, intakeRollerSubsystem);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeRollerSubsystem.setSpeed(IntakeRollerConstants.kStop);
    m_PrimerSubsystem.setSpeed(PrimerConstants.kStop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_PrimerSubsystem.isPrimerBeamBreakBroken() && !m_IntakeRollerSubsystem.intakeHasPiece());
  }
}


