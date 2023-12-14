// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.SwerveModuleConstants;

public class SwerveModule {

  private SwerveModuleConstants module_constants;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveMotorEncoder;
  private final RelativeEncoder m_turningMotorEncoder;
  private final SparkMaxAbsoluteEncoder m_angleEncoder;

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  // For tuning only. Use a simple pid P-Only controller to get the value for kP.
  // Then we can work on Ka and Ks (trapezoidal constraints)
  private final PIDController m_simpleTurningPIDController = new PIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0);

  /**
   * Constructs a SwerveModule.
   *
   * @param module_constants The module constants (i.e. CAN IDs) for this specific
   *                         module
   */
  public SwerveModule(SwerveModuleConstants module_constants) {
    this.module_constants = module_constants;

    m_driveMotor = new CANSparkMax(module_constants.driveMotorID, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(module_constants.angleMotorID, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_turningMotor.restoreFactoryDefaults();

    m_driveMotorEncoder = m_driveMotor.getEncoder();
    m_turningMotorEncoder = m_turningMotor.getEncoder();

    m_angleEncoder = m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);

    configureDevices();
    resetEncoders();
  }

  /**
   * Returns current turn position in range -pi to pi
   */
  public double getTurningPosition() {
    return m_turningMotorEncoder.getPosition();
  }

  public double getTurningVelocity() {
    return m_turningMotorEncoder.getVelocity();
  }

  /**
   * Get the Drive motor encoder position
   *
   * @return the drive encoder position
   */
  public double getDrivePosition() {
    return m_driveMotorEncoder.getPosition();
  }

  /**
   * Get the drive wheel velocity
   *
   * @return drive encoder velocity
   */
  public double getDriveVelocity() {
    return m_driveMotorEncoder.getVelocity();
  }

  /**
   * Get the raw value from the absolute encoder on the SparkMax
   *
   * @return raw angle (0.0->1.0)
   */
  public double getRawAngle() {
    return m_angleEncoder.getPosition();
  }

  /**
   * Return the rotation vector for the absolute module angular position
   *
   * @return double representing the absolute module angle
   */

  public double getAbsoluteEncoderRad() {
    double angle = 2.0 * Math.PI * getRawAngle();
    // angle -= absoluteEncoderOffsetRad;
    return angle * (module_constants.angleEncoderReversed ? -1.0 : 1.0);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveMotorEncoder.setPosition(0.0);
    m_turningMotorEncoder.setPosition(getAbsoluteEncoderRad());
  }

  /**
   * Returns the current state (velocity/angle) of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double velocity = getDriveVelocity();
    return new SwerveModuleState(velocity, new Rotation2d(getTurningPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    double distance = getDrivePosition();
    Rotation2d rot = new Rotation2d(getTurningPosition());
    return new SwerveModulePosition(distance, rot);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getState().angle);
    // SwerveModuleState state = desiredState;

    double driveOutput = (desiredState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput_trap = m_turningPIDController.calculate(
        m_angleEncoder.getPosition(),
        state.angle.getRadians());
    final double turnOutput = m_simpleTurningPIDController.calculate(
        m_angleEncoder.getPosition(),
        state.angle.getRadians());

    SmartDashboard.putNumber("Swerve[" + m_turningMotor.getDeviceId() + "] setpoint", m_turningPIDController.getSetpoint().position);
    SmartDashboard.putNumber("Swerve[" + m_turningMotor.getDeviceId() + "] driveOutput", driveOutput);
    SmartDashboard.putNumber("Swerve[" + m_turningMotor.getDeviceId() + "] turnOutput", turnOutput);
    SmartDashboard.putNumber("Swerve[" + m_turningMotor.getDeviceId() + "] Trapezoidal turnOutput", turnOutput_trap);

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);

    SmartDashboard.putString("Swerve[" + m_turningMotor.getDeviceId() + "] state", state.toString());
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }

  /*
   * Configuration values for the module hardware. Specifically the motors and
   * encoders.
   */
  private void configureDevices() {
    // Drive motor configuration:
    // - L2 NEO Motor connected to SParkMax
    // - SDS4i model L2 has a drive motor to wheel ratio of 6.75:1
    // and an adjusted speed of 14.5 ft/sec with the NEO motors
    // - Outside wheel diameter = 4in
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.clearFaults();
    if (m_driveMotor.setIdleMode(Constants.ModuleConstants.DRIVE_IDLE_MODE) != REVLibError.kOk) {
      SmartDashboard.putString("Drive Motor Idle Mode", "Error");
    }
    m_driveMotor.setInverted(module_constants.driveMotorReversed);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveMotorEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    m_driveMotorEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    // Angle motor configuration.
    // Neo Motor connected to SParkMax (all turn motors are reversed in the SDS 4i)
    // The steering gear ration (from turning motor to wheel) is 150/7:1 = 21.43
    m_turningMotor.restoreFactoryDefaults();
    m_turningMotor.clearFaults();
    if (m_turningMotor.setIdleMode(Constants.ModuleConstants.ANGLE_IDLE_MODE) != REVLibError.kOk) {
      SmartDashboard.putString("Turn Motor Idle Mode", "Error");
    }
    m_turningMotor.setInverted(module_constants.angleMotorReversed);
    m_turningMotor.setSmartCurrentLimit(
        Constants.ModuleConstants.ANGLE_CURRENT_LIMIT);
    m_driveMotorEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    m_driveMotorEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    /**
     * CTRE Mag Encoder connected to the SparkMAX Absolute/Analog/PWM Duty Cycle
     * input
     * Native will ready 0.0 -> 1.0 for each revolution.
     */
    m_angleEncoder.setZeroOffset(module_constants.angleEncoderOffset);
    m_angleEncoder.setInverted(module_constants.angleEncoderReversed);
    m_angleEncoder.setAverageDepth(16);

    /**
     * Make PID continuous around the 180degree point of the rotation
     */
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turningPIDController.setTolerance(1.0);

    m_simpleTurningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_simpleTurningPIDController.setTolerance(1.0);
  }
}
