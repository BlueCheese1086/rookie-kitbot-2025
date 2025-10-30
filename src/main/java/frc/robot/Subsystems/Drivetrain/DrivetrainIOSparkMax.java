// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.RobotMap;

/** Add your docs here. */
public class DrivetrainIOSparkMax extends DrivetrainIO {
    SparkMax leftFrontTalon = new SparkMax(RobotMap.LEFT_FRONT_TALON_ID, MotorType.kBrushless);
    SparkMax rightFrontTalon = new SparkMax(RobotMap.RIGHT_FRONT_TALON_ID, MotorType.kBrushless);
    SparkMax leftBackTalon = new SparkMax(RobotMap.LEFT_BACK_TALON_ID, MotorType.kBrushless);
    SparkMax rightBackTalon = new SparkMax(RobotMap.RIGHT_BACK_TALON_ID, MotorType.kBrushless);
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    public void setVoltagesArcadeCommand(double left, double right) {
    }
    public void setVoltages(double left, double right) {
        leftFrontTalon.setVoltage(left);
        rightFrontTalon.setVoltage(right);
        System.out.println(right);
        System.out.println(left);
    }
    public DrivetrainIOSparkMax() {

    }

}
