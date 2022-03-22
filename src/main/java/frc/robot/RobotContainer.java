// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.HolonomicDriveCommand;
import frc.robot.commands.RunPath;
import frc.robot.subsystems.HolonomicDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final HolonomicDrive driveBase = new HolonomicDrive();

  private XboxController driver = new XboxController(0);

  double t = 0;
  double tInc;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    t = 0;
    tInc = .0001;

    driveBase.setDefaultCommand(
        new HolonomicDriveCommand(this::getXSpeed, this::getYSpeed, this::getRotationSpeed, driveBase));
  }

  // The three doubles below represent the values from our controller
  public double getXSpeed() {
    return driver.getLeftY();
  }

  public double getYSpeed() {
    return driver.getLeftX();
  }

  public double getRotationSpeed() {
    return driver.getRightX();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(driver, XboxController.Button.kA.value)
        .whenPressed(new SequentialCommandGroup(new RunPath(0, 0, 30, driveBase), new WaitCommand(5),
            new RunPath(0.25, 0, 0, driveBase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new RunPath(0, 0, Math.toRadians(0), driveBase);
    // return null;
  }

}
