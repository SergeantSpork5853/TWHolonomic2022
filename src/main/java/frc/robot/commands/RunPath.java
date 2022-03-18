// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.JetsonRpLidar;
import frc.robot.subsystems.HolonomicDrive;

public class RunPath extends CommandBase {
  /** Creates a new RunPath. */
  public double desiredX;
  public double desiredY;
  public double desiredTheta;

  double t;

  double tInc;

  public PIDController xController;
  public PIDController yController;
  public PIDController thetaController;

  public HolonomicDrive driveBase = new HolonomicDrive();
  public JetsonRpLidar jetsonLidar = new JetsonRpLidar();

  public RunPath(double desiredX, double desiredY, double desiredTheta, HolonomicDrive driveBase) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.desiredX = desiredX;
    this.desiredY = desiredY;
    this.desiredTheta = desiredTheta;
    this.driveBase = driveBase;

    this.addRequirements(driveBase);

    xController = new PIDController(0, 0, 0);
    yController = new PIDController(0, 0, 0);
    thetaController = new PIDController(2, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(0.05);
    yController.setTolerance(0.05);
    thetaController.setTolerance(0.3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    t = 0;

    tInc = 0.001;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentXValue = jetsonLidar.getX();
    double currentYValue = jetsonLidar.getY();
    double currentThetaValue = jetsonLidar.getTheta();

    driveBase.drive(xController.calculate(currentXValue, desiredX),
        yController.calculate(currentYValue, desiredY),
        -thetaController.calculate(currentThetaValue, ramp()));
    ramp();
    SmartDashboard.putNumber("PID ERROR", Math.toDegrees(thetaController.getPositionError()));
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

  public double ramp() {
    double plim = 30;
    double mlim = -30;
    t = t + tInc;
    if (t > plim) {
      tInc = -tInc;
    }
    if (t < mlim) {
      tInc = -tInc;
    }

    SmartDashboard.putNumber("Current", t);

    return t;
  }
}
