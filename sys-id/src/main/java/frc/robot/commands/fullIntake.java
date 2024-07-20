package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.Constants.IntakeWristConstants;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeWristSubsystem;

public class fullIntake extends Command{
    private IntakeRollerSubsystem intakeRollerSubsystem;
    private IntakeWristSubsystem intakeWristSubsystem;
    public fullIntake(IntakeRollerSubsystem intakeRollerSubsystem , IntakeWristSubsystem intakeWristSubsystem) {
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        this.intakeWristSubsystem = intakeWristSubsystem;
    }
    @Override
    public void initialize() {
        intakeRollerSubsystem.setSpeed(IntakeRollerConstants.kIntake);
        intakeWristSubsystem.setRequest(IntakeWristConstants.kIntake);
    }

    @Override
    public void end(boolean interrupted) {
        intakeRollerSubsystem.setSpeed(IntakeRollerConstants.kHold);
        intakeWristSubsystem.setRequest(IntakeWristConstants.kStow);
    }

    @Override
    public boolean isFinished() {
        return intakeRollerSubsystem.intakeHasPiece() && intakeWristSubsystem.isAtReqPosition(IntakeWristConstants.kIntake); //shouldn't do anything ideally;
    }

    
}
