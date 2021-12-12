// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MecanumDriveSubsystem;
import frc.robot.Constants;

public class MoveByDistanceCommand extends CommandBase {

  MecanumDriveSubsystem mecanumDriveSubsystem;
  double setpoint, error;

  /** Creates a new MoveByDistanceCommand. */
  public MoveByDistanceCommand(MecanumDriveSubsystem mecanumDriveSubsystem, double setpoint) {
    this.mecanumDriveSubsystem = mecanumDriveSubsystem;
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mecanumDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.encR.reset();
    RobotContainer.encL.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.error = this.setpoint - ((this.mecanumDriveSubsystem.getAverageDistance()) * Constants.encoderScale);
    double correction = this.error * 0.2;
    this.mecanumDriveSubsystem.moveByDistance(correction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Math.abs(this.angleError) <= Constants.deadband && Math.abs(this.distanceError) <= Constants.deadband
    return (Math.abs(this.error) <= Math.max(0.01d, (setpoint * Constants.deadband)));
  }
}
