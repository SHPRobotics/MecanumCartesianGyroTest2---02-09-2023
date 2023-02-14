// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleOldPIDCmd extends CommandBase {
  private final PIDController turnController = new PIDController(0.01, 0.0, 0.0);
  private final DriveSubsystem m_subsystem;
  private final double m_targetAngleDegrees;

  /** Creates a new TurnToAngleOldPIDCmd. */
  public TurnToAngleOldPIDCmd(double targetAngleDegrees, DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    m_targetAngleDegrees = targetAngleDegrees;
    // set setpoint
    turnController.setSetpoint(targetAngleDegrees);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentRotationRate = turnController.calculate(m_subsystem.getGyro().getAngle());
    m_subsystem.getMecanumDrive().driveCartesian(0.0, 0.0, currentRotationRate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
