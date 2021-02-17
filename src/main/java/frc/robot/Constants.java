// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final int kRightMotor1Port = 3;
    public static final int kRightMotor2Port = 4; 
    
    public static final int PigeonIMUPort = 0;

    public static final TalonFXInvertType kLeftInvertType = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType kRightInvertType = TalonFXInvertType.CounterClockwise;

    public static final double kDriveSpeed = 0.8;

    //void - need to delte (Falcon motors have integrated encoders)
    /*public static final int[] kLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kRightEncoderPorts = new int[] {2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;*/
    //void

    public static final double kTrackwidthMeters = Units.inchesToMeters(27); //27 inches
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 2048; //https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html
    public static final double kWheelDiameterMeters = 0.15; //6 inches
    public static final double kGearReduction = 10.71;
    public static final double kEncoderDistancePerPulse =
        ((kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR) / kGearReduction; 
    public static final double kWheelDistancePerPulse = kEncoderDistancePerPulse/ kGearReduction; //DISTANCE PER PULSE OF WHEEL= (OUTER CIRCUMFERENCE OF WHEEL)/(ENCODER CPR*GEAR REDUCTION)

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot. 
    
  
    //THE CONSTANTS BELOW NEED TO BE TESTED FOR THE VOLTAGE EQUATION (*FEEDFORWARD CONSTANTS*)
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5; // THE "P" *FEEDBACK* CONSTANT
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final Button kToggleQuickTurnButton = Button.kA;
    public static final Button kIntakeButton = Button.kA;
    public static final Button kStopRollerButton = Button.kX;
    public static final Button kOuttakeButton = Button.kB;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public final class IntakeConstants {
    public static final int kArmPort = 5;
    public static final int kRollerPort = 6;

    public static final boolean kArmInvert = false;
    public static final boolean kRollerInvert = false;

    public static final double kIntakeSpeed = 0.8;
    public static final double kOuttakeSpeed = 0.8;
 }
}
