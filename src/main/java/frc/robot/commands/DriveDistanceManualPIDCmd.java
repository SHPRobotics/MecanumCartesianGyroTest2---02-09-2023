// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceManualPIDCmd extends CommandBase {
  private final double m_setpointFt;
  private final DriveSubsystem m_subsystem;

  private double errorSum = 0.0;
  private double lastError = 0.0;
  private double lastTimestamp = Timer.getFPGATimestamp();
  private final double iLimit = 1.0;

  /** Creates a new DriveDistanceManualPIDCmd. */
  public DriveDistanceManualPIDCmd(double setpointFt, DriveSubsystem subsystem) {
    m_setpointFt = setpointFt;
    m_subsystem = subsystem;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get sensor position
    double sensorPosition = Units.metersToFeet(m_subsystem.getAverageEncoderDistance());

    // calculate the error
    double error = m_setpointFt - sensorPosition;

    // calculate time change
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(error) < iLimit) errorSum += error * dt;

    double errorRate = (error - lastError) / dt;

    // calculate the output
    double outputSpeed = DriveConstants.kDistP * error
                        + DriveConstants.kDistI * errorSum
                        + DriveConstants.kDistD * errorRate;

    System.out.println("Sensor Position Ft: "+ sensorPosition 
                        + ", error Ft: " + error
                        + ", errorSum Ft: " + errorSum
                        + ", errorRate Ft: " + errorRate
                        + ", outputSpeed: " + outputSpeed);

    // drive the robot with the calculated output
    m_subsystem.drive(outputSpeed, 0, 0);

    // update last- variables
    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.getAverageEncoderDistance()) >= Math.abs(Units.feetToMeters(m_setpointFt));
  }
}
