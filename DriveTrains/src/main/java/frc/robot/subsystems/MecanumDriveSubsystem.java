// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class MecanumDriveSubsystem extends SubsystemBase {

  private final WPI_TalonSRX FR;
  private final WPI_TalonSRX BR;
  private final WPI_TalonSRX FL;
  private final WPI_TalonSRX BL;  

  private final MecanumDrive mecanumDriveTrain;

  public Translation2d FLLocation;
  public Translation2d FRLocation;
  public Translation2d BLLocation;
  public Translation2d BRLocation;

  /** Creates a new MecanumDriveSubsystem. */
  public MecanumDriveSubsystem() {
    FR = new WPI_TalonSRX(Constants.FR_port);
    BR = new WPI_TalonSRX(Constants.BR_port);
    
    FL = new WPI_TalonSRX(Constants.FL_port);
    BL = new WPI_TalonSRX(Constants.BL_port); 

    // Translation2d FLLocation = new Translation2d(0.381, 0.381);
    // Translation2d FRLocation = new Translation2d(0.381, -0.381);
    // Translation2d BLLocation = new Translation2d(-0.381, 0.381);
    // Translation2d BRLocation = new Translation2d(-0.381, -0.381);
    
    mecanumDriveTrain = new MecanumDrive(FL, BL, FR, BR);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Joy Y", RobotContainer.getY(RobotContainer.joy1, Constants.deadband));
    SmartDashboard.putNumber("Joy Z", RobotContainer.getY(RobotContainer.joy1, Constants.deadband));

  }

  public void mecanumArcade(double y, double x, double z) {
    mecanumDriveTrain.driveCartesian(x * Constants.maxSpeed, y * Constants.maxSpeed, z * Constants.maxSpeed);
  }

  public void brakeMode() {
    FL.setNeutralMode(NeutralMode.Brake);
    FR.setNeutralMode(NeutralMode.Brake);
    BL.setNeutralMode(NeutralMode.Brake);
    BR.setNeutralMode(NeutralMode.Brake);
  }

  public void coastMode() {
    FL.setNeutralMode(NeutralMode.Coast);
    FR.setNeutralMode(NeutralMode.Coast);
    BL.setNeutralMode(NeutralMode.Coast);
    BR.setNeutralMode(NeutralMode.Coast);
  }

  public void drive(double l, double r) {
    FR.set(r);
    BR.set(r);
    FL.set(l);
    BL.set(l);
  }

  public void driveRaw(double y) {
    drive(y * Constants.maxSpeed, y * Constants.maxSpeed);
  }

  public double getAverageDistance() {
    return (RobotContainer.encL.getDistance() + RobotContainer.encR.getDistance())/2;
  }

  public void moveByDistance(double correction) {
    if (Math.abs(correction) < Constants.minSpeed) correction = Math.signum(correction) * Constants.minSpeed;
    if (Math.abs(correction) > Constants.maxSpeed) correction = Math.signum(correction) * Constants.maxSpeed;
    drive(correction, correction);
  }

  public void moveByAngle(double correction) {
    if (Math.abs(correction) < Constants.turnMinSpeed) correction = Math.signum(correction) * Constants.turnMinSpeed;
    if (Math.abs(correction) > Constants.turnMaxSpeed) correction = Math.signum(correction) * Constants.turnMaxSpeed;
    drive(correction, -correction);
  }

  public void swerve(double angleCorrection, double distanceCorrection) {
    double correctionLeft = distanceCorrection + angleCorrection;
    double correctionRight =  distanceCorrection - angleCorrection;

    if (Math.abs(correctionLeft) < Constants.minSpeed) correctionLeft = Math.signum(correctionLeft) * Constants.minSpeed;
    if (Math.abs(correctionLeft) > Constants.maxSpeed) correctionLeft = Math.signum(correctionLeft) * Constants.maxSpeed;
    if (Math.abs(correctionRight) < Constants.minSpeed) correctionRight = Math.signum(correctionRight) * Constants.minSpeed;
    if (Math.abs(correctionRight) > Constants.maxSpeed) correctionRight = Math.signum(correctionRight) * Constants.maxSpeed;

    drive(correctionLeft, correctionRight);
  }

}
