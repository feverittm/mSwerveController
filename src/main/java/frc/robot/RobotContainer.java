// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


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
		m_driverController.x().onTrue(turnToCommand(Math.PI/2.0));

		SmartDashboard.putBoolean("Field Mode", m_robotDrive.getFieldMode());

		// need to create a toggle for field oriented drive mode.
		// m_driverController.leftBumper().toggleOnTrue(getAutonomousCommand());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return new RunCommand(
				() -> m_robotDrive.drive(0, 0, 0, false));
	}

	public Command turnToCommand(double rot) {
		return new RunCommand(
				() -> m_robotDrive.drive(0, 0, rot, false));
	}

	public void setDriveMode() {
		m_robotDrive.setDefaultCommand(new SwerveJoystickCmd(
			m_robotDrive,
			() -> -m_driverController.getRawAxis(OIConstants.kDriverYAxis),
			() -> m_driverController.getRawAxis(OIConstants.kDriverXAxis),
			() -> m_driverController.getRawAxis(OIConstants.kDriverRotAxis),
			m_robotDrive.getFieldMode()
		));

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