// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  // OperatorConstants ========================================================================================================================
  public static class OperatorConstants {
    public static final int kLeftJoystickPort = 0;
    public static final int  kRightJoystickPort =1;
    public static final int kXboxControllerPort = 2;
    public static final double kDeadband = 0.05;

    public static final int kOffBalanceAngleThresholdDegrees = 3;
    public static final int kOnBalanceAngleThresholdDegrees = 1;

    public static final int kJoystickTurnTo90 = 1;
    public static final int kJoystickGyroReset = 2;
    public static final int kJoystickBalanceRobot = 3;
  }

  // DriveConstants ===========================================================================================================================
  public final class DriveConstants{
    // Drive's Motors DeviceID
    public static final int kfrontLeftMotorDeviceID = 1;
    public static final int krearLeftMotorDeviceID  =2;
    public static final int kfrontRightMotorDeviceID =3;
    public static final int krearRightMotorDeviceID =4;

    // Max. speed the robot can drive
    public static final double kMaxOutput = 1.00; //0.75;

    // Conversion factor from tick (pulse) to meter
    public static final double kWheelDiameterMeters = 0.152;     // 6" (or 0.152 meter) diameter of robot's wheel
    public static final int kEncoderCPR = 42;                    // Encoder Counts Per Revolution (https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf)
    public static final double kEncoderGearRatio = 4.16;         // 4.16;
    // kEncoderCPR x 1 tick = 1 revolution = (kWheelDiameterMeters x π) x GearRatio
    //           (kWheelDiameterInches x π)                
    // 1 tick = ──────────────────────────── x GearRatio 
    //                  kEncoderCPR                        
    //
    public static final double kEncoderDistancePerPulseMeters =
        (kWheelDiameterMeters * Math.PI) / kEncoderCPR * kEncoderGearRatio;

    public static final boolean kGyroReversed = false;

    public static final double kTurnP = 0.5;
    public static final double kTurnI = 0.0;    //0.5;
    public static final double kTurnD = 0;

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10;

    public static final double kDistP = 0.5;    
    public static final double kDistI = 0.0;
    public static final double kDistD = 0.0;


  }

  public final class ArmConstants{
    public static final int kArmExtenderID = 6;
    public static final int kArmGripID = 7;
    public static final int kArmLiftID = 5;
    // test speeds of all arm motors
    public static final double kExtenderSpeed = 0.5;
    public static final double kArmSpeed = 0.5;
    public static final double kGripSpeed = 0.5;

  }

  // AutoConstants ============================================================================================================================
  public final class AutoConstants{

    public static final double kautoSpeed = 0.5;
    // The following PID Controller coefficients will need to be tuned for your drive system.
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0.0;
    public static final double kaVoltSecondsPerMeter = 0.0;
    public static final double kPDriveVel = 0.0;
    // horizontal distance between the left and right wheels in meters
    public final double kTrackwidthMeters = Units.inchesToMeters(27);
    public final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

  }

public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 1.35;
public static final double BEAM_BALANACED_DRIVE_KP = 0.01; //0.015;
public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
}
