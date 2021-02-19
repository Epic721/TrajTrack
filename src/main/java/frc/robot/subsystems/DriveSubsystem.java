package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  
  //DEVICES/SENSORS
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(Constants.DriveConstants.kLeftMotor1Port);
  private final WPI_TalonFX leftFollower = new WPI_TalonFX(Constants.DriveConstants.kLeftMotor2Port);
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(Constants.DriveConstants.kRightMotor1Port);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(Constants.DriveConstants.kRightMotor2Port);

  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(leftMaster, leftFollower);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(rightMaster, rightFollower);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The gyro sensor
  //private final PigeonIMU m_gyro = new PigeonIMU(Constants.DriveConstants.PigeonIMUPort);
  private final ADIS16448_IMU m_gyro = new ADIS16448_IMU(); //NOTE: THIS GYRO MEASURES CONTINUOUSLY (0,1,2,...,360,361,362,...) -> SHOULDN'T MATTER SINCE THE POSE2D is converted to degrees by the WPILib methods

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;





  /** Creates a new DriveSubsystem. */ //CONSTRUCTOR
  public DriveSubsystem() {
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }





  //PERIODIC
  @Override
  public void periodic() {
    // Continually update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), getWheelDistance(leftMaster), getWheelDistance(rightMaster));
  }





  //FUNCTIONS
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() { //Pose = Position+Heading
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {//USED TO UPDATE RAMSETE CONTROLLER (IN ROBOT CONTAINER) WITH THE SPEEDS SO THAT IT CAN PERFORM PID ON THEM 
    return new DifferentialDriveWheelSpeeds(getWheelVelocity(leftMaster), getWheelVelocity(rightMaster));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) { //FOR SETTING THE ODOMETRY TO THE INITIAL POSE AT THE START OF THE TRAJECTORY BEFORE STARTING TO TRAVERSE IT (IN ROBOTCONTAINER)
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed(); 
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() { //ISN'T BEING USED ANYWHERE ATM
    return (leftMaster.getSelectedSensorPosition() + rightMaster.getSelectedSensorPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */

  
  //WE DON'T NEED THIS
  // public Encoder getLeftEncoder() {
  //   return m_leftEncoder;
  // }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */

  //WE DON'T NEED THIS
  // public Encoder getRightEncoder() {
  //   return m_rightEncoder;
  // }


  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) { //CAN BE USED TO ADD FUNCTIONS FOR HYPERDRIVE
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() { //YAW FROM POSE2D
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate(); //WHY NEGATIVE?->ANS:UNIT CIRCLE IS OPPOSITE OF INTUITIVE ROBOT DIRECTION
  }

  
  //CUSTOM DEFINED FUNCTIONS (FOR PIGEON IMU CLASS (NOT THE ADI...))
  /*public Rotation2d getRotation2dPdg(){
    double YPR [] = new double [3];
    m_gyro.getYawPitchRoll(YPR);
    return Rotation2d.fromDegrees(YPR[0]);
  }

  public double getRatePdg(){
    double XYZ [] = new double [3];
    m_gyro.getRawGyro(XYZ);
    return XYZ[0];
  }*/

  double getWheelDistance(WPI_TalonFX encoderOfFalcon){
    return encoderOfFalcon.getSelectedSensorPosition()*Constants.DriveConstants.kWheelDistancePerPulse; //# of counts-> m
  }

  double getWheelVelocity(WPI_TalonFX encoderOfFalcon){
    return encoderOfFalcon.getSelectedSensorVelocity()*Constants.DriveConstants.kWheelDistancePerPulse/60; //rpm-> m/s
  }
}