// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	private final DriveSubsystem m_robotDrive = new DriveSubsystem();

	// The driver's controller
	CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure default commands
		setDriveMode();

		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this method to define bindings between conditions and commands. These
	 * are useful for
	 * automating robot behaviors based on button and sensor input.
	 *
	 * <p>
	 * Should be called during {@link Robot#robotInit()}.
	 *
	 * <p>
	 * Event binding methods are available on the {@link Trigger} class.
	 */
	private void configureButtonBindings() {
		m_driverController.a().onTrue(m_robotDrive.zeroModules());
		m_driverController.b().onTrue(m_robotDrive.ResetEncoders());
		m_driverController.y().onTrue(m_robotDrive.SetFieldMode());
		m_driverController.x().onTrue(moveTurnCommand(Math.PI/2.0));

		// need to create a toggle for field oriented drive mode.
		m_driverController.leftBumper().toggleOnTrue(getAutonomousCommand())
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	// public Command getAutonomousCommand() {
	// // Create config for trajectory
	// TrajectoryConfig config = new TrajectoryConfig(
	// AutoConstants.kMaxSpeedMetersPerSecond,
	// AutoConstants.kMaxAccelerationMetersPerSecondSquared)
	// // Add kinematics to ensure max speed is actually obeyed
	// .setKinematics(DriveConstants.kDriveKinematics);

	// // An example trajectory to follow. All units in meters.
	// Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
	// // Start at the origin facing the +X direction
	// new Pose2d(0, 0, new Rotation2d(0)),
	// // Pass through these two interior waypoints, making an 's' curve path
	// List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
	// // End 3 meters straight ahead of where we started, facing forward
	// new Pose2d(3, 0, new Rotation2d(0)),
	// config);

	// var thetaController = new ProfiledPIDController(
	// AutoConstants.kPThetaController, 0, 0,
	// AutoConstants.kThetaControllerConstraints);
	// thetaController.enableContinuousInput(-Math.PI, Math.PI);

	// SwerveControllerCommand swerveControllerCommand = new
	// SwerveControllerCommand(
	// exampleTrajectory,
	// m_robotDrive::getPose, // Functional interface to feed supplier
	// DriveConstants.kDriveKinematics,

	// // Position controllers
	// new PIDController(AutoConstants.kPXController, 0, 0),
	// new PIDController(AutoConstants.kPYController, 0, 0),
	// thetaController,
	// m_robotDrive::setModuleStates,
	// m_robotDrive);

	// // Reset odometry to the starting pose of the trajectory.
	// m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

	// // Run path following command, then stop at the end.
	// return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
	// false));
	// }
	public Command getAutonomousCommand() {
		return new RunCommand(
				() -> m_robotDrive.drive(0, 0, 0, false));
	}

	public Command moveTurnCommand(double rot) {
		return new RunCommand(
				() -> m_robotDrive.drive(0, 0, rot, false));
	}

	public void setDriveMode() {
		        m_robotDrive.setDefaultCommand(new SwerveJoystickCmd(
                m_robotDrive,
                () -> -m_driverController.getRawAxis(OIConstants.kDriverYAxis),
                () -> m_driverController.getRawAxis(OIConstants.kDriverXAxis),
                () -> m_driverController.getRawAxis(OIConstants.kDriverRotAxis),
                () -> false));

		// Configure default commands
		// m_robotDrive.setDefaultCommand(
		// 		// The left stick controls translation of the robot.
		// 		// Turning is controlled by the X axis of the right stick.
		// 		new RunCommand(
		// 				() -> m_robotDrive.drive(
		// 						MathUtil.applyDeadband(m_driverController.getLeftY(),
		// 								Constants.OIConstants.LEFT_Y_DEADBAND),
		// 						MathUtil.applyDeadband(m_driverController.getLeftX(),
		// 								Constants.OIConstants.LEFT_X_DEADBAND),
		// 						-m_driverController.getRightX(),
		// 						false),
		// 				m_robotDrive));
	}
}