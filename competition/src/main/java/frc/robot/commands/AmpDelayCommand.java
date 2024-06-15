package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PrimerConstants;
import frc.robot.Constants.ShooterWristConstants;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class AmpDelayCommand extends Command {
    ShooterPivotSubsystem pivot;
    boolean isDone;
    BooleanSupplier isPrimerBeambreakBroken;

    public AmpDelayCommand(ShooterPivotSubsystem pivotSubsystem, BooleanSupplier isBeambreakBroken) {
        pivot = pivotSubsystem;
        isPrimerBeambreakBroken = isBeambreakBroken;
    }

    @Override
    public void execute() {
        if (!isPrimerBeambreakBroken.getAsBoolean()) {
            try {
                Thread.sleep((long)(PrimerConstants.kAmpDelay * 1000));
            } catch (Exception e) {}
            pivot.setRequest(ShooterWristConstants.kFlat);
        }
    }
}
