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
        lastAngle = getState().angle.getRadians();
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_driveMotorEncoder.setPosition(0.0);
        m_turningMotorEncoder.setPosition(0.0);
    }

    /*
     * Set the drive motor brake mode
     */
    public void setMotorBrake(boolean brakeMode) {
        IdleMode setbrake = brakeMode ? IdleMode.kBrake : IdleMode.kCoast;
        if (m_driveMotor.setIdleMode(setbrake) != REVLibError.kOk) {
            SmartDashboard.putString("Drive Motor Idle Mode", "Error");
        }
    }

    /*
     * Linear map function
     * Copied from the Arduino source code
     * (Creative Commons Attribution-Share Alike 3.0 License.)
     */
    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**
     * Get the raw encoder position which is natively 0-1.
     * 
     * @return raw encoder position (native 0-1)
     */
    public double getRawAbsoluteEncoder() {
        double _angle = m_angleEncoder.getPosition();
        SmartDashboard.putNumber("Raw Absolute Encoder: ", _angle);
        return _angle;
    }

    /*
     * Read the absolute encoder.
     * This is tricky. The encoder is a CTRE mag encoder that is has the analog
     * output connected to the duty-cycle input on the SparkMax motor controller.
     * The native output is 0.0 -> 1.0
     * What we want is in radians -pi -> 0 -> pi and is continuous (wraps around)
     */
    public double getAbsoluteEncoder() {
        double output = map(getRawAbsoluteEncoder(), 0.0, 1.0, -Math.PI, Math.PI);
        return output;
    }

    public Rotation2d getAngle() {
        return new Rotation2d(getAbsoluteEncoder());
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double velocity = m_driveMotorEncoder.getVelocity();
        return new SwerveModuleState(velocity, getAngle());
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
        //SwerveModuleState state = SwerveModuleState.optimize(desiredState, getState().angle);
        SwerveModuleState state = desiredState;

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(m_driveMotorEncoder.getVelocity(),
                state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(getAbsoluteEncoder(), state.angle.getRadians());

        SmartDashboard.putNumber("turnOutput", turnOutput);
        SmartDashboard.putNumber("driveOutput", driveOutput);

        // Calculate the turning motor output from the turning PID controller.
        m_driveMotor.set(driveOutput);
        m_turningMotor.set(turnOutput);
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
        m_driveMotor.clearFaults();
        if (m_driveMotor.setIdleMode(Constants.ModuleConstants.DRIVE_IDLE_MODE) != REVLibError.kOk) {
            SmartDashboard.putString("Drive Motor Idle Mode", "Error");
        }
        m_driveMotor.setInverted(module_constants.driveMotorReversed);
        m_driveMotor.setSmartCurrentLimit(Constants.ModuleConstants.DRIVE_CURRENT_LIMIT);

        m_driveMotorEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);
        m_driveMotorEncoder.setPosition(0);

        // Angle motor configuration.
        // Neo Motor connected to SParkMax (all turn motors are reversed in the SDS 4i)
        // The steering gear ration (from turning motor to wheel) is 150/7:1 = 21.43
        m_turningMotor.clearFaults();
        if (m_turningMotor.setIdleMode(Constants.ModuleConstants.ANGLE_IDLE_MODE) != REVLibError.kOk) {
            SmartDashboard.putString("Turn Motor Idle Mode", "Error");
        }
        m_turningMotor.setInverted(module_constants.angleMotorReversed);
        m_turningMotor.setSmartCurrentLimit(Constants.ModuleConstants.ANGLE_CURRENT_LIMIT);

        m_turningMotorEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderDistancePerPulse);
        m_turningMotorEncoder.setPosition(0);

        /**
         * CTRE Mag Encoder connected to the SparkMAX Absolute/Analog/PWM Duty Cycle
         * input
         * Native will ready 0.0 -> 1.0 for each revolution.
         * will use an internal getAbsoluteEncoder() method to scale the data from the
         * sensor.
         */
        m_angleEncoder.setInverted(module_constants.angleEncoderReversed);
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

}
