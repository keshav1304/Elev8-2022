// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MecanumDriveSubsystem;

public class MecanumDriveCommand extends CommandBase {

  MecanumDriveSubsystem mecanumDriveSubsystem;

  /** Creates a new DriveCommand. */
  public MecanumDriveCommand(MecanumDriveSubsystem mecanumDriveSubsystem) {
    this.mecanumDriveSubsystem = mecanumDriveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mecanumDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yaxis = RobotContainer.getY(RobotContainer.joy2, Constants.deadband); 
    double zaxis = RobotContainer.getZ(RobotContainer.joy2, Constants.deadband); 
    mecanumDriveSubsystem.arcadeInbuilt(yaxis, 0.5, zaxis);
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
