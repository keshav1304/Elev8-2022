// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  public WPI_VictorSPX shooter;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooter = new WPI_VictorSPX(Constants.shooterPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shooter2(double y){
    shooter.set(y);
  }
}
