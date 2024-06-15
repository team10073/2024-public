package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.AnalogInput;

public class BeamBreak {
    /** Analog input for detecting beam breaks. */
  private AnalogInput primerBeambreak;

    public BeamBreak() { //Helper class that needs to change 
        primerBeambreak = new AnalogInput(ShooterConstants.kShooterAnalogInputChannel);
    }

    /**
   * Returns a BooleanSupplier representing if the beambreak has been broken or not.
   */
    public BooleanSupplier isPrimerBeamBreakBroken() {
        return () -> ((Math.floor(primerBeambreak.getVoltage()) > 0));
    }

}
