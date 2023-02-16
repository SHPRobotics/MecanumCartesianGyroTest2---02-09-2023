// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmExtenderSubsystem extends SubsystemBase {
  /** Creates a new ArmExtenderSubsystem. */
  private  final CANSparkMax ArmExtenderMotor = new CANSparkMax(Constants.ArmConstants.kArmExtenderID, MotorType.kBrushless);
  public ArmExtenderSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setMotor(double speed) {
    ArmExtenderMotor.set(speed);
  }
}
