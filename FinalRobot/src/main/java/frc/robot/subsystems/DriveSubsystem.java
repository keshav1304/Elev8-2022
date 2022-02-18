// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {

  private final Talon FR;
  private final Talon BR;
  private final MotorControllerGroup rightSide;

  private final Talon FL;
  private final Talon BL;  
  private final MotorControllerGroup leftSide;

  private final DifferentialDrive driveTrain;

  private double acceleration;
  private double velocity;
  private double displacement;
  private double previousTime;
  private Timer timer;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    // timer = new Timer();
    // resetIntegrals();
    // timer.reset();
    // timer.start();

    FR = new Talon(Constants.FR_port);
    BR = new Talon(Constants.BR_port);
    rightSide = new MotorControllerGroup(FR, BR);
    
    FL = new Talon(Constants.FL_port);
    BL = new Talon(Constants.BL_port); 
    leftSide = new MotorControllerGroup(FL, BL);
    
    driveTrain = new DifferentialDrive(leftSide, rightSide);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // updateDistance();

    // SmartDashboard.putNumber("Displacement", this.displacement);
    // SmartDashboard.putNumber("Velocity", this.velocity);
    // SmartDashboard.putNumber("Acceleration", this.acceleration);

    //SmartDashboard.putNumber("Nav X", RobotContainer.navx.getYaw());
    //SmartDashboard.putNumber("Enc R", RobotContainer.encR.getDistance() * Constants.rightScale);
    //SmartDashboard.putNumber("Enc L", RobotContainer.encL.getDistance());
    SmartDashboard.putNumber("Joy Y", RobotContainer.getY(RobotContainer.joy1, Constants.deadband));
    SmartDashboard.putNumber("Joy Z", RobotContainer.getY(RobotContainer.joy1, Constants.deadband));

  }

  public void arcadeInbuilt(double y, double z) {
    FR.setInverted(false);
    BR.setInverted(false);
    driveTrain.arcadeDrive(y * Constants.maxSpeed, z * Constants.maxSpeed);
  }

  public void drive(double l, double r) {
    FR.setInverted(true);
    BR.setInverted(true);

    FR.set(r);
    BR.set(r);
    FL.set(l);
    BL.set(l);
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
