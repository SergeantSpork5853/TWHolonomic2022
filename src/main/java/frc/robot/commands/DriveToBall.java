// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.RPLidar;
import frc.robot.subsystems.HolonomicDrive;

public class DriveToBall extends CommandBase {
  /** Creates a new DriveToBall. */
  public RPLidar lidar = new RPLidar();
  public HolonomicDrive driveBase;

  public PIDController xController;
  public PIDController yController;
  public PIDController thetaController;

  public DriveToBall(HolonomicDrive driveBase) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveBase = driveBase;
    this.addRequirements(driveBase);

    xController = new PIDController(0, 0, 0);
    yController = new PIDController(0, 0, 0);
    thetaController = new PIDController(1, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double currentXValue = lidar.getX();
    // double currentYValue = lidar.getY();
    // double currentThetaValue = lidar.getTheta();

    // driveBase.drive(xController.calculate(currentXValue, 0),
    // yController.calculate(currentYValue, 0),
    // thetaController.calculate(currentThetaValue, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
