// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private  final CANSparkMax armLiftMotor = new CANSparkMax(Constants.ArmConstants.kArmLiftID, MotorType.kBrushless);
  private final RelativeEncoder m_armEncoder = armLiftMotor.getEncoder();

  public  ArmSubsystem() {
    m_armEncoder.setPositionConversionFactor(ArmConstants.kArmEncoderTick2Degs);
    // set brake mode
    armLiftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  /*
    // set softLimits
    armLiftMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kArmRotationForwardLimit);
    armLiftMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.kArmRotationReverseLimit);

    // enable them
    armLiftMotor.enableSoftLimit(SoftLimitDirection.kForward , true);
    armLiftMotor.enableSoftLimit(SoftLimitDirection.kReverse , true);
  */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Encoder Value Deg", m_armEncoder.getPosition());
  }

  public void setMotor(double speed) {
    armLiftMotor.set(speed);
 }

 public void resetArmEncoder(){
  m_armEncoder.setPosition(0);
 }
}
