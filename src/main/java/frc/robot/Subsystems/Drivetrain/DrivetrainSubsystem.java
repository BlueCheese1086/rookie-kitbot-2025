// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  public static final int LEFT_FRONT_TALON_ID = 1;
  public static final int RIGHT_FRONT_TALON_ID = 2;
  public static final int LEFT_BACK_TALON_ID = 3;
  public static final int RIGHT_BACK_TALON_ID = 4;

  TalonSRX leftFrontTalon = new TalonSRX(LEFT_FRONT_TALON_ID);
  TalonSRX rightFrontTalon = new TalonSRX(RIGHT_FRONT_TALON_ID);
  TalonSRX leftBackTalon = new TalonSRX(LEFT_BACK_TALON_ID);
  TalonSRX rightBackTalon = new TalonSRX(RIGHT_BACK_TALON_ID);
  TalonSRXConfiguration config = new TalonSRXConfiguration();
  VoltageOut leftVoltage = new VoltageOut(0);
  VoltageOut rightVoltage = new VoltageOut(0);

  private void setVoltages(double left, double right) {
    leftFrontTalon.set(ControlMode.PercentOutput, left);
    rightFrontTalon.set(ControlMode.PercentOutput, right);
    System.out.println(right);
    System.out.println(left);
  }

  public Command setVoltagesCommand(DoubleSupplier left, DoubleSupplier right) {
    return this.run(() -> this.setVoltages(left.getAsDouble(), right.getAsDouble()));
  }

  public Command setVoltagesArcadeCommand(DoubleSupplier drive, DoubleSupplier steer) {
    System.out.println("Command: SetVoltagesArcadeCommand Running.");
    return this.run(() -> {
      System.out.println("Command: setVoltagesArcadeCommand 2 Running.");
      var speeds = DifferentialDrive.arcadeDriveIK(drive.getAsDouble(), steer.getAsDouble(), false);
      this.setVoltages(speeds.left, speeds.right);
    });

  }

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    leftBackTalon.follow(leftFrontTalon);
    rightBackTalon.follow(rightFrontTalon);
    rightFrontTalon.setInverted(true);
    rightBackTalon.setInverted(true);

    leftBackTalon.configPeakCurrentLimit(80, 2);
    leftFrontTalon.configPeakCurrentLimit(80, 2);
    rightBackTalon.configPeakCurrentLimit(80, 2);
    rightFrontTalon.configPeakCurrentLimit(80, 2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
