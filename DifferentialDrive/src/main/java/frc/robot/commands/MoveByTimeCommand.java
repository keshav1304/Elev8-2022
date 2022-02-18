// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;

public class MoveByTimeCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  double setpoint, error;
  double i = 0;

  /** Creates a new MoveByTimeCommand. */
  public MoveByTimeCommand(DriveSubsystem driveSubsystem, double setpoint) {

    this.driveSubsystem = driveSubsystem;
    this.setpoint = setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.error = this.setpoint - i*0.02;
    this.error = this.setpoint - RobotContainer.navx.getDisplacementX() * Constants.encoderScale;
    double correction = this.error * 0.2;
    this.driveSubsystem.moveByDistance(correction);
    i++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
