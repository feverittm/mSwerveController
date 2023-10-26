// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.SwerveModuleConstants;

public class SwerveModule {
    private SwerveModuleConstants module_constants;

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final RelativeEncoder m_driveMotorEncoder;
    private final RelativeEncoder m_turningMotorEncoder;
    private final SparkMaxAbsoluteEncoder m_angleEncoder;

    public double lastAngle;

    private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            ModuleConstants.kPModuleTurningController,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                    ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

    /**
     * Constructs a SwerveModule.
     *
     * @param module_constants The module constants (i.e. CAN IDs) for this specific module
     */
    public SwerveModule(SwerveModuleConstants module_constants) {
        this.module_constants = module_constants;

        m_driveMotor = new CANSparkMax(module_constants.driveMotorID, MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(module_constants.angleMotorID, MotorType.kBrushless);

        m_driveMotorEncoder = m_driveMotor.getEncoder();
        m_turningMotorEncoder = m_turningMotor.getEncoder();

        m_angleEncoder = m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);


        configureDevices();
        lastAngle = getState().angle.getRadians();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double velocity = m_driveMotorEncoder.getVelocity();
        Rotation2d rot = new Rotation2d(m_angleEncoder.getPosition());
        return new SwerveModuleState(velocity, rot);
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        double distance = m_driveMotorEncoder.getPosition();
        Rotation2d rot = new Rotation2d(m_angleEncoder.getPosition());
        return new SwerveModulePosition(distance, rot);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getState().angle);

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(m_driveMotorEncoder.getVelocity(), state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(m_angleEncoder.getPosition(), state.angle.getRadians()
                - Units.degreesToRadians(module_constants.angleEncoderOffsetDegrees));

        // Calculate the turning motor output from the turning PID controller.
        m_driveMotor.set(driveOutput);
        m_turningMotor.set(turnOutput);
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_driveMotorEncoder.setPosition(0.0);
        m_turningMotorEncoder.setPosition(0.0);
    }

    /*
     * Configuration values for the module hardware.  Specifically the motors and encoders.
     */
    private void configureDevices() {
        // Drive motor configuration:
        //  - L2 NEO Motor connected to SParkMax
        //  - SDS4i model L2 has a drive motor to wheel ratio of 6.75:1
        // and an adjusted speed of 14.5 ft/sec with the NEO motors
        //  - Outside wheel diameter = 4in
        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.clearFaults();
        if (m_driveMotor.setIdleMode(IdleMode.kCoast) != REVLibError.kOk) {
            SmartDashboard.putString("Drive Motor Idle Mode", "Error");
        }

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_driveMotorEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);

        // Set whether drive encoder should be reversed or not
        m_driveMotor.setInverted(module_constants.driveMotorReversed);

        // Set the distance (in this case, angle) in radians per pulse for the turning
        // encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        m_turningMotorEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderDistancePerPulse);

        // Set whether turning encoder should be reversed or not
        m_turningMotor.setInverted(module_constants.angleMotorReversed);

        m_driveMotor.setInverted(module_constants.driveMotorReversed);
        m_driveMotor.setIdleMode(Constants.ModuleConstants.DRIVE_IDLE_MODE);
        m_driveMotorEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderDistancePerPulse);
        m_driveMotorEncoder.setPosition(0);

        // Angle motor configuration.
        // Neo Motor connected to SParkMax (all turn motors are reversed in the SDS 4i)
        // The steering gear ration (from turning motor to wheel) is 150/7:1 = 21.43
        m_turningMotor.restoreFactoryDefaults();
        m_turningMotor.clearFaults();
        if (m_turningMotor.setIdleMode(IdleMode.kCoast) != REVLibError.kOk) {
            SmartDashboard.putString("Turn Motor Idle Mode", "Error");
        }
        m_turningMotor.setInverted(module_constants.angleMotorReversed);
        m_turningMotor.setIdleMode(Constants.ModuleConstants.ANGLE_IDLE_MODE);
        m_turningMotor.setSmartCurrentLimit(Constants.ModuleConstants.ANGLE_CURRENT_LIMIT);

        /**
         * CTRE Mag Encoder connected to the SparkMAX Absolute/Analog/PWM Duty Cycle
         * input
         * Native will ready 0.0 -> 1.0 for each revolution.
         */
        m_angleEncoder.setPositionConversionFactor(ModuleConstants.kAngleEncodeAnglePerRev);
        m_angleEncoder.setPositionConversionFactor(Constants.ModuleConstants.kAngleEncodeAnglePerRev);
        m_angleEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kAngleEncodeAnglePerRev);
        m_angleEncoder.setInverted(false);

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

}
