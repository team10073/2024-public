package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.IntakeWristSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.Constants.IntakeWristConstants;

public class TeleopIntakeToPrimerCommand extends SequentialCommandGroup{
    public TeleopIntakeToPrimerCommand(IntakeRollerSubsystem intakeRollers, IntakeWristSubsystem intakeWrist, 
    ShooterPivotSubsystem pivot, IndexerSubsystem indexer, PrimerSubsystem primer, BeamBreak beambreak) {
    addCommands(
        indexer.stopCommand(),
        primer.stopCommand(),
        pivot.goToIntakePos(),
        intakeWrist.intakePosCommand(), 
        intakeRollers.intakeCommand(),
        intakeWrist.indexPosCommand(),
        intakeRollers.holdCommand().until(() -> intakeWrist.isAtReqPosition(IntakeWristConstants.kIntake)),
        new ParallelDeadlineGroup(
          primer.intakeCommand().until(beambreak.isPrimerBeamBreakBroken()),
          intakeRollers.outtakeCommand(),
          indexer.forwardCommand()
        ),
        pivot.goToAmpPose()
        );
    }
}
