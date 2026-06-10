// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rangefinder extends SubsystemBase {
  /** Creates a new Rangefinder. */
  XRPRangefinder m_rangefinder;
  public Rangefinder() {
    m_rangefinder=new XRPRangefinder();
  }

  public double getDistanceMeters(){
    return m_rangefinder.getDistanceMeters();
  }

  public double getDistanceInches(){
    return m_rangefinder.getDistanceInches();
  }

  public Distance getDistanceMetersMeasure() {
    return Meters.of(getDistanceMeters());
  }

  public Distance getDistanceAsUnit(DistanceUnit unit){
    //edu.wpi.first.units.Units.Meters;
    
    return unit.of(unit.convertFrom(getDistanceMeters(), Meters));
  }


  @Override
  public void periodic() {}
}
