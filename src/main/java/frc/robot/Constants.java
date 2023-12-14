// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        public static final class DriveConstants {
                /** Constants that apply to the whole drive train. */
                public static final double kTrackWidth = Units.inchesToMeters(24.0); // Width of the drivetrain measured
                                                                                     // from the middle of the wheels.
                public static final double kWheelBase = Units.inchesToMeters(24.0); // Length of the drivetrain measured
                                                                                    // from the middle of the wheels.
                public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
                public static final double WHEEL_CIRCUMFERENCE = kWheelDiameterMeters * Math.PI;

                // Distance between front and back wheels on robot
                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

                public static final boolean kGyroReversed = false;

                // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
                // These characterization values MUST be determined either experimentally or
                // theoretically
                // for *your* robot's drive.
                // The SysId tool provides a convenient method for obtaining these values for
                // your robot.
                public static final double ksVolts = 1;
                public static final double kvVoltSecondsPerMeter = 0.8;
                public static final double kaVoltSecondsSquaredPerMeter = 0.15;

                public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
                public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

                public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
                public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                        kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
                public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
                public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        }

        public static final class ModuleConstants {

                public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
                public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

                public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
                public static final double kDriveMotorGearRatio = 1 / 6.75;
                public static final int kTurnMotorEncoderTicksPerRotation = 42;
                public static final double kTurningMotorRotationPerSteerRotation = 1 / 21.4; // 150/7:1
                public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
                public static final double kTurningEncoderRot2Rad = kTurningMotorRotationPerSteerRotation
                    * kTurnMotorEncoderTicksPerRotation;
                // public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio;
                public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
                public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

                /** Idle modes. */
                public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
                public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;

                /** Current limiting. */
                public static final int DRIVE_CURRENT_LIMIT = 35;
                public static final int ANGLE_CURRENT_LIMIT = 25;

                /** Module PID Kp constants */
                public static final double kPModuleTurningController = 0.5;
                public static final double kPModuleDriveController = 0.0020645;

                /**
                 * Module specific constants.
                 * CanCoder offset is in DEGREES, not radians like the rest of the repo.
                 * This is to make offset slightly more accurate and easier to measure.
                 */
                // Front Left Module
                public static final SwerveModuleConstants kMOD_1_Constants = new SwerveModuleConstants(
                                1,
                                8,
                                1,
                                true,
                                true,
                                false,
                                0.28 // 254.5 degrees = 360 * 0.060
                );

                // Front Right
                public static final SwerveModuleConstants kMOD_2_Constants = new SwerveModuleConstants(
                                2,
                                6,
                                7,
                                false,
                                true,
                                false,
                                0.48 // 0.7069 // 152.0 degrees = 360 * 0.7069
                );

                // Back Left
                public static final SwerveModuleConstants kMOD_4_Constants = new SwerveModuleConstants(
                                4,
                                2,
                                3,
                                true,
                                true,
                                false,
                                0.7 // 0.324 // 131.0 degrees = 360 * 0.324
                );

                // Back Right
                public static final SwerveModuleConstants kMOD_3_Constants = new SwerveModuleConstants(
                                3,
                                4,
                                5,
                                false,
                                true,
                                false,
                                0.202 // 0.4221 // 152.0 degrees = 360 * 0.4221
                );
        }

        public static final class OIConstants {

                public static final int kDriverControllerPort = 0;

                public static final int kDriverYAxis = 1;
                public static final int kDriverXAxis = 0;
                public static final int kDriverRotAxis = 4;
                public static final int kDriverFieldOrientedButtonIdx = 1;
        
                public static final int GYRO_RESET_BUTTON = XboxController.Button.kA.value;
                public static final int ENCODER_RESET_BUTTON = XboxController.Button.kB.value;

                // Joystick Deadband
                public static final double kDeadband = 0.05;
        }

        public static final class AutoConstants {
                public static final double kMaxSpeedMetersPerSecond = 3;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;
                public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
                public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

                public static final double kPXController = 1;
                public static final double kPYController = 1;
                public static final double kPThetaController = 1;

                // Constraint for the motion profiled robot angle controller
                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        }
}
