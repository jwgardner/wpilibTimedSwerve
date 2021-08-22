// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;  

// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;

// JWG add Talon libraries
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// JWG

public class SwerveModule {
  // JWG don't need, no drive motor encoder
  // private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  //JWG talon SRX raw encoder position needs to be converted to radians for Rotation2d
  private static final double kTicksPerDegree = kEncoderResolution / 360;

  private static final double kModuleMaxAngularVelocity = SwerveDrivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  // private final MotorController m_driveMotor;
  // private final MotorController m_turningMotor;
  private WPI_TalonSRX m_driveMotor;
  private WPI_TalonSRX m_turningMotor;

  // private final Encoder m_driveEncoder;
  // private final Encoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  // JWG not used, we have no drive encoder
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  // JWG Parameters are from example representing ks - static gain and vs - velocity gain
  // JWG feedforward outputs for a simple permanent-magnet DC motor
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param driveEncoderChannelA DIO input for the drive encoder channel A
   * @param driveEncoderChannelB DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  // JWG dont have encoder channels, using TalonSRX sensor feedback from analog 5v encoder
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel)
      // int driveEncoderChannelA,
      // int driveEncoderChannelB,
      // int turningEncoderChannelA,
      // int turningEncoderChannelB) 
      {
    
    // m_driveMotor = new PWMSparkMax(driveMotorChannel);
    // m_turningMotor = new PWMSparkMax(turningMotorChannel);
    m_driveMotor = new WPI_TalonSRX(driveMotorChannel);
    m_turningMotor = new WPI_TalonSRX(turningMotorChannel);

    // m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
    // m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    
    // JWG reading encoder through TalonSRX trying getSelectedSensorPosition(0), where 0 is primary closed loop
    double degrees = m_turningMotor.getSelectedSensorPosition(0) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    
    // JWG **** we dont have an encoder rate for current speed, can we maintain a variable that holds equivalent velocity based on last voltage set ?  use zero for now
    // return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.get()));
    return new SwerveModuleState(0.0, new Rotation2d(radians));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees

    // JWG Rotation2d expecting radians, what is getSelectedSensorPosition returning
    // JWG reading encoder through TalonSRX trying getSelectedSensorPosition(0), where 0 is primary closed loop        
    double degrees = m_turningMotor.getSelectedSensorPosition(0) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    SwerveModuleState state =
        // SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));
        SwerveModuleState.optimize(desiredState, new Rotation2d(radians));

    // JWG dont have drive encoder for output, can simulate m_driveEncoder.getRate() ????  use zero for now
    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
    // m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);
        m_drivePIDController.calculate(0.0, state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
     // JWG reading encoder through TalonSRX trying getSelectedSensorPosition(0), where 0 is primary closed loop 
     // m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());
        m_turningPIDController.calculate(m_turningMotor.getSelectedSensorPosition(0), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}