package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;

  public LEDSubsystem() {
    led = new AddressableLED(LEDConstants.PWMPortLeft);
    ledBuffer = new AddressableLEDBuffer(LEDConstants.LEDLength);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public void setLEDToShooter() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, LEDConstants.cubeHValue, LEDConstants.cubeSValue,
          LEDConstants.cubeVValue);
    }
    led.setData(ledBuffer);
  }

  public void setToHue(int hue) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, hue, 255, 130);
    }
    led.setData(ledBuffer);
  }

  public void setLedtoIntake() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, LEDConstants.coneHValue, LEDConstants.coneSValue,
          LEDConstants.coneVValue);
    }
    led.setData(ledBuffer);
  }

  public void setLedOff() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
  }
  public Command setLedIntakeCommand(){
    return run(() -> setLedtoIntake()).finallyDo(() -> setLedOff());
  }
  public Command setLedShooter() {
    return run(() -> setLEDToShooter()).finallyDo(() -> setLedOff());
  }
  public Command turnOffLed(){
    return runOnce(() -> setLedOff());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("led", ledBuffer.getLength());
  }

}
