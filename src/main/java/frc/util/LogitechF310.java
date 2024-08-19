package frc.util;

import edu.wpi.first.wpilibj.Joystick;

public class LogitechF310 extends Joystick {

  public LogitechF310(int port) {
    super(port);
    super.setXChannel(0);
    super.setYChannel(1);
  }

  public double getRotation(){
    return this.getRawAxis((LogitechAxis.RX.getValue()));
  }


  
}