package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;



public class handleLEDCommand extends Command {
    private LEDSubsystem led;
    private BooleanSupplier intake;
    private BooleanSupplier shooter;

    public handleLEDCommand(LEDSubsystem led, BooleanSupplier intake, BooleanSupplier shooter ) {
        this.led = led;
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(led);
    }

    @Override
    public void initialize() {
        led.setLedOff();
    }
    @Override
    public void execute() {
        if(intake.getAsBoolean()) {
            led.setLedtoIntake();
        } else if (shooter.getAsBoolean()) {
            led.setLEDToShooter();
        } else {
            led.setLedOff();
        }
    }
    
}
