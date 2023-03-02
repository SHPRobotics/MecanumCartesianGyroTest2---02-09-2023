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
import frc.robot.Constants.GripConstants;

public class GripperSubsystem extends SubsystemBase {
  /** Creates a new GripperSubsystem. */
  private  final CANSparkMax armGripMotor = new CANSparkMax(GripConstants.kArmGripID, MotorType.kBrushless);
    // Grip encoder
  public final RelativeEncoder m_gripEncoder = armGripMotor.getEncoder();

  public boolean isSetLimitsCube = false;
  public boolean isSetLimitsCone = false;

  public GripperSubsystem() {
    // conversion factor from tick to meters
    // m_gripEncoder.setPositionConversionFactor(GripConstants.kGripEncoderTick2Meters);

    //setSoftLimitsCube();;
  }    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("GripEncoder value ", m_gripEncoder.getPosition());
  
  }

  public void setMotor(double speed) {
    armGripMotor.set(speed);
  }

  public void resetGripEncoder(){
    m_gripEncoder.setPosition(0);
   }

  public void grabCube(){
    if (!isSetLimitsCube) setSoftLimitsCube();
    setMotor(GripConstants.kGripSpeed);
  }

  public void grabCone(){
    if (!isSetLimitsCone) setSoftLimitsCone();
    setMotor(GripConstants.kGripSpeed);
  }

  public void setSoftLimitsCone(){
    disableSoftLimits();

    //set softLimits cone
    armGripMotor.setSoftLimit(SoftLimitDirection.kForward, 85.0f); //minimum val for cube to be squished
    armGripMotor.setSoftLimit(SoftLimitDirection.kReverse, -85.0f); // max val for gripper to  be extended
    
    enableSoftLimits();

    isSetLimitsCone = true;
    isSetLimitsCube = false;
  }

  public void setSoftLimitsCube(){
    // if gripper is to the left of Cube's min limit, bring the gripper at 0 position before setting the softLimits
    if (m_gripEncoder.getPosition() > 0) m_gripEncoder.setPosition(0) ;

    disableSoftLimits();

    //set softLimits for the cube
    armGripMotor.setSoftLimit(SoftLimitDirection.kForward, 0.0f); //minimum val for cube to be squished
    armGripMotor.setSoftLimit(SoftLimitDirection.kReverse, -85.0f); // max val for gripper to  be extended
    
    enableSoftLimits();

    isSetLimitsCube = true;
    isSetLimitsCone = false;

  }

  public void disableSoftLimits(){  
    armGripMotor.enableSoftLimit(SoftLimitDirection.kForward , false);
    armGripMotor.enableSoftLimit(SoftLimitDirection.kReverse , false);
  }

  public void enableSoftLimits(){  
    armGripMotor.enableSoftLimit(SoftLimitDirection.kForward , true);
    armGripMotor.enableSoftLimit(SoftLimitDirection.kReverse , true);
  }

} // End of public class GripperSubsystem
