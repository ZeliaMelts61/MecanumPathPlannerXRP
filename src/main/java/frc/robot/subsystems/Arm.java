// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final XRPServo m_armServo;
  private final XRPServo m_armServo2;
  private boolean inverted = false;
  /** Creates a new Arm. */
  public Arm() {
    // Device number 4 maps to the physical Servo 1 port on the XRP
    m_armServo = new XRPServo(5);
    m_armServo2 = new XRPServo(4);
    inverted = ArmConstants.kInverted;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Set the current angle of the arm (0 - 180 degrees).
   *
   * @param angleDeg Desired arm angle in degrees
   */
  public void setAngle(double angleDeg) {
    
    m_armServo.setAngle((inverted ? 180-angleDeg : angleDeg));
    m_armServo2.setAngle((inverted ? 180-angleDeg : angleDeg));
    
  }
  public void setPosition(double amount){
    m_armServo.setPosition((inverted ? 1.0-amount : amount));
    m_armServo2.setPosition((inverted ? 1.0-amount : amount));

  }
  public double getPosition(){
    return (inverted ? 1.0-m_armServo.getPosition() : m_armServo.getPosition());
  }

  public double getAngle(){
    return (inverted ? 180-m_armServo.getAngle() : m_armServo.getAngle());
  }
  public void printPosition(){
    System.out.println(m_armServo.getPosition());
  }

}
