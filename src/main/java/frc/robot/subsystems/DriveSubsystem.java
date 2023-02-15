// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  WPI_TalonFX frontLeftController, frontRightController, backLeftController, backRightController;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  frontLeftController = new WPI_TalonFX(DriveConstants.LEFT_FRONT_TALON);
  frontRightController = new WPI_TalonFX(DriveConstants.LEFT_FRONT_TALON);
  backLeftController = new WPI_TalonFX(DriveConstants.LEFT_FRONT_TALON);
  backRightController = new WPI_TalonFX(DriveConstants.LEFT_FRONT_TALON);

  frontLeftController.setInverted(TalonFXInvertType.Clockwise);
  backLeftController.setInverted(TalonFXInvertType.Clockwise);
  frontRightController.setInverted(TalonFXInvertType.Clockwise);
  backRightController.setInverted(TalonFXInvertType.Clockwise);
  
  frontLeftController.configFactoryDefault();
  backLeftController.configFactoryDefault();
  frontRightController.configFactoryDefault();
  backRightController.configFactoryDefault();

  }
  @Override
  
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void diffDrive(double speed, double rotation){
    frontRightController.set(ControlMode.PercentOutput, -speed,DemandType.ArbitraryFeedForward, rotation);
    backRightController.follow(frontLeftController);
    frontLeftController.set(ControlMode.PercentOutput, speed,DemandType.ArbitraryFeedForward, rotation);
    backLeftController.follow(frontLeftController);
  }
}


