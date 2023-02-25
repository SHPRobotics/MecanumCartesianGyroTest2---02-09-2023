// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripConstants;

public class GripperSubsystem extends SubsystemBase {
  /** Creates a new GripperSubsystem. */
  private  final CANSparkMax armGripMotor = new CANSparkMax(GripConstants.kArmGripID, MotorType.kBrushless);
    // Grip encoder
  private final RelativeEncoder m_gripEncoder = armGripMotor.getEncoder();

  public GripperSubsystem() {
       // conversion factor from tick to meters
       m_gripEncoder.setPositionConversionFactor(GripConstants.kGripEncoderTick2Meters);
       // set softLimit ?
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("GripEncoder value Meter", m_gripEncoder.getPosition());
  
  }

  public void setMotor(double speed) {
    armGripMotor.set(speed);
  }
}
