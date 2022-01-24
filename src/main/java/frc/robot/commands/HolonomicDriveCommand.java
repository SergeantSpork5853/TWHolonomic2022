// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HolonomicDrive;

public class HolonomicDriveCommand extends CommandBase { 
  private HolonomicDrive driveBase; 
  private DoubleSupplier getXSpeed; 
  private DoubleSupplier getYSpeed; 
  private DoubleSupplier getRotationSpeed; 

  private final double DEADBAND = 0.4;

  /** Creates a new HolonomicDriveController. */
  public HolonomicDriveCommand(DoubleSupplier getXSpeed, 
                                  DoubleSupplier getYSpeed, 
                                  DoubleSupplier getRotationSpeed, 
                                  HolonomicDrive driveBase) {
    // Use addRequirements() here to declare subsystem dependencies. 
    this.getXSpeed = getXSpeed; 
    this.getYSpeed = getYSpeed; 
    this.getRotationSpeed = getRotationSpeed; 
    this.driveBase = driveBase; 
    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = deadband(getXSpeed.getAsDouble()); 
    double ySpeed = deadband(getYSpeed.getAsDouble()); 
    double rotationSpeed = deadband(getRotationSpeed.getAsDouble()); 

    driveBase.drive(xSpeed, ySpeed, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.drive(0.0, 0.0, 0.0);
  } 

  public double deadband(double value){ 
    if (Math.abs(value) < DEADBAND) return 0.0; 
    return value; 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
