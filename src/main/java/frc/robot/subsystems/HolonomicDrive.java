// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//CTRE dependencies
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//NavX (our gyro sensor) dependencies
import com.kauailabs.navx.frc.AHRS;
//WPILIB dependencies
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.JetsonRpLidar;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.RPLidar;

public class HolonomicDrive extends SubsystemBase {

  double t;
  double tInc;

  /**
   * This value is "dummy" for now. It should be meters per second but we are
   * using percent voltage control so it instead
   * represents 100% voltage
   */
  public static final double maxSpeed = 1;
  // This value also goes unused for now. I am not sure if we will need to use it
  // even when we use velocity control
  public static final double maxAngularSpeed = Math.PI;
  // The CAN bus IDs of each motor in the holonomic drive base
  private final WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(1);
  private final WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(2);
  private final WPI_TalonSRX backLeftMotor = new WPI_TalonSRX(3);
  private final WPI_TalonSRX backRightMotor = new WPI_TalonSRX(4);
  // The approximate location of each holonomic wheel
  private final Translation2d frontLeftLocation = new Translation2d(Units.inchesToMeters(11), Units.inchesToMeters(10));
  private final Translation2d frontRightLocation = new Translation2d(Units.inchesToMeters(11),
      Units.inchesToMeters(-10));
  private final Translation2d backLeftLocation = new Translation2d(Units.inchesToMeters(-11), Units.inchesToMeters(10));
  private final Translation2d backRightLocation = new Translation2d(Units.inchesToMeters(-11),
      Units.inchesToMeters(-10));
  // Our gyro sensor that returns the robot heading. It is plugged into the serial
  // port on the robot
  public AHRS gyro = new AHRS(SPI.Port.kMXP);
  // This kinematics constructor will allow use to command the three separate
  // motors as a unified drivebase
  private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(frontLeftLocation, frontRightLocation,
      backLeftLocation, backRightLocation);

  private RPLidar myLidar = new RPLidar();

  private JetsonRpLidar myJetsonRpLidar = new JetsonRpLidar();

  private Limelight myCamera = new Limelight();

  /** Creates a new HolonomicDrive. */
  public HolonomicDrive() {
    // So far, resetting the gyro sensor every robot boot-up has not presented any
    // problems
    gyro.reset();
    /**
     * One half of the drivebase must be inverted to drive in non-field oriented
     * mode. I am not sure if it is
     * also necessary for field oriented mode
     */
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

    myCamera.setPipeline((byte) 0);
  }

  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    // Once again, the "meters per second" component is treated like a percent
    // voltage since it is limited to -1 to 1
    final double frontLeftOutput = speeds.frontLeftMetersPerSecond;
    final double frontRightOutput = speeds.frontRightMetersPerSecond;
    final double backLeftOutput = speeds.rearLeftMetersPerSecond;
    final double backRightOutput = speeds.rearRightMetersPerSecond;

    // Here you can see where the motors are each commanded to a unique percent
    // voltage value defined above
    frontLeftMotor.set(ControlMode.PercentOutput, frontLeftOutput);
    frontRightMotor.set(ControlMode.PercentOutput, frontRightOutput);
    backLeftMotor.set(ControlMode.PercentOutput, backLeftOutput);
    backRightMotor.set(ControlMode.PercentOutput, backRightOutput);
  }

  // Our drive method. It takes three independent speeds from our joystick and
  // blends them to get one directional speed
  public void drive(double xSpeed, double ySpeed, double rot) {
    // "var" is not the best way to do this, but the example code did it so I didn't
    // change it
    var mecanumDriveWheelSpeeds =
        // Here is where the kinematics come into play
        kinematics.toWheelSpeeds(
            // Depending on the value of this boolean, we will either drive field oriented
            // or robot oriented
            fieldRelative()
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                    /**
                     * I originally had `Rotation2d.fromDegrees(gyro.getAngle())`, but for some
                     * reason this affected field
                     * oriented strafing. However, the below function works fine. Not sure what is
                     * wrong here
                     */
                    gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    mecanumDriveWheelSpeeds.desaturate(maxSpeed);
    setSpeeds(mecanumDriveWheelSpeeds);

    t = 0.0;
    tInc = 0.001;

  }

  // Checks if we have a gyro sensor plugged in, enabling field oriented if it is
  // and disabling it if it is not
  public boolean fieldRelative() {
    return (gyro != null) ? true : false;
  }

  @Override
  public void periodic() {
    myLidar.publishData();
    myJetsonRpLidar.publishData();
  }

}
