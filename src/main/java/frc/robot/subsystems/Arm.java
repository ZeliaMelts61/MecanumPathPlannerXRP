// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
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
    angleDeg=MathUtil.clamp(angleDeg, 0, 180);
    setPosition(angleDeg / 180.0);    
  }
  public void setPosition(double amount){
    amount=scale((inverted ? 1.0-amount : amount));
    m_armServo.setPosition(amount);
    m_armServo2.setPosition(amount);

  }
  public double getPosition(){
    double truePos = (inverted ? 1.0-m_armServo.getPosition() : m_armServo.getPosition());
    return unscale(truePos);
  }

  public double getAngle(){
    return getPosition()*180;
  }
  public void printPosition(){
    System.out.println(m_armServo.getPosition());
  }

  private static double scale(double value) {
    double outMin=ArmConstants.kMinOutput;
    double outMax=ArmConstants.kMaxOutput;
    value = MathUtil.clamp(value, 0.0, 1.0);
    return (value - 0.0) * (outMax - outMin) / (1.0 - 0.0) + outMin;
  }

  private static double unscale(double value) {
    double outMin = ArmConstants.kMinOutput;
    double outMax = ArmConstants.kMaxOutput;
    value = MathUtil.clamp(value, outMin, outMax);
    return (value - outMin) / (outMax - outMin);
  }

}
