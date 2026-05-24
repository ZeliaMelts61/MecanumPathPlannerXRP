// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.xrp.XRPOnBoardIO;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rsl extends SubsystemBase {
  /** Creates a new Rsl. */

  XRPOnBoardIO m_onboardIO;

  public Rsl(XRPOnBoardIO onboardIO) {
    this.m_onboardIO=onboardIO;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setRSL(boolean on){
    m_onboardIO.setLed(on);
  }

  public boolean getRSL() {
    return m_onboardIO.getLed();
  }

  public void turnOn(){
    setRSL(true);
  }

  public void turnOff(){
    setRSL(false);
  }

  public void toggle(){
    setRSL(!getRSL());
  }
}
