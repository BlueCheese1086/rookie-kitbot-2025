// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.RobotMap;

/** Add your docs here. */
public class DrivetrainIOTalonSRX extends DrivetrainIO{
    TalonSRX leftFrontTalon = new TalonSRX(RobotMap.LEFT_FRONT_TALON_ID);
    TalonSRX rightFrontTalon = new TalonSRX(RobotMap.RIGHT_FRONT_TALON_ID);
    TalonSRX leftBackTalon = new TalonSRX(RobotMap.LEFT_BACK_TALON_ID);
    TalonSRX rightBackTalon = new TalonSRX(RobotMap.RIGHT_BACK_TALON_ID);
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    @Override
    public void setVoltagesArcadeCommand(double left, double right) {
    }
    public void setVoltages(double left, double right) {
        leftFrontTalon.set(ControlMode.PercentOutput, left);
        rightFrontTalon.set(ControlMode.PercentOutput, right);
        System.out.println(right);
        System.out.println(left);
      }
   public DrivetrainIOTalonSRX() {
    leftBackTalon.follow(leftFrontTalon);
    rightBackTalon.follow(rightFrontTalon);
    rightFrontTalon.setInverted(true);
    rightBackTalon.setInverted(true);

    leftBackTalon.configPeakCurrentLimit(80, 2);
    leftFrontTalon.configPeakCurrentLimit(80, 2);
    rightBackTalon.configPeakCurrentLimit(80, 2);
    rightFrontTalon.configPeakCurrentLimit(80, 2);
   }


}
