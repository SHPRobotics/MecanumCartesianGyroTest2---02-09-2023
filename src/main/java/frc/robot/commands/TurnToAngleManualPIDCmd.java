// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleManualPIDCmd extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final double m_setpointDeg;
  private double errorSum = 0.0;
  private double lastError = 0.0;
  private double lastTimestamp = Timer.getFPGATimestamp();
  private final double iLimit = 1.0;

  /** Creates a new TurnToAngleCmd. */
  public TurnToAngleManualPIDCmd(double setpointDeg, DriveSubsystem driveSubsystem) {
    m_setpointDeg = setpointDeg;
    m_driveSubsystem = driveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get sensor angle, ie. current heading in degrees
    double sensorAngle = m_driveSubsystem.getHeading();
    SmartDashboard.putNumber("getHeading Deg", sensorAngle);

    // calculate the error
    double error = m_setpointDeg - sensorAngle;

    // calculate time change
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(error) < iLimit) errorSum += error * dt;
    double errorRate = (error - lastError) / dt;

    // calculate the output
    double output = DriveConstants.kTurnP * error
                    + DriveConstants.kTurnI * errorSum
                    + DriveConstants.kTurnD * errorRate;
    ;
    SmartDashboard.putNumber("output", output);

    System.out.println("Sensor Position Ft: "+ sensorAngle
                        + ", error Ft: " + error
                        + ", errorSum Ft: " + errorSum
                        + ", errorRate Ft: " + errorRate
                        + ", outputSpeed: " + output);

    // turn the robot
    m_driveSubsystem.drive(0, 0, output);

    // update last- variables
    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_driveSubsystem.getGyro().getAngle()) >= Math.abs(m_setpointDeg);
  }
}
