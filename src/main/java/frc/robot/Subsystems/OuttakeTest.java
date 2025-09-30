// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.io.ObjectInputFilter.Config;
import java.util.function.DoubleSupplier;
import com.revrobotics.spark.SparkBase.PersistMode; 
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
   
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class OuttakeTest extends SubsystemBase {
  // Plug in USB to C into Spark MAX on bot and open rev software to check and change ID. 
  public static final int SPARKMAX_OUTTAKE_ID = 5;

  SparkMax outtakeMotor = new SparkMax(SPARKMAX_OUTTAKE_ID, SparkMax.MotorType.kBrushed);

  private void setVoltages(double volt) {    
    outtakeMotor.setVoltage(volt);
    System.out.println(volt);
  }

  public Command setVoltagesCommand(DoubleSupplier volt) {
    return this.run(() -> this.setVoltages(volt.getAsDouble()));
  }
  public OuttakeTest() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    outtakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  
  }
   public Command setOuttakeVoltagesArcadeCommand(DoubleSupplier drop){
    System.out.println("Command: setOuttakeVoltagesArcadeCommand: Running. ");
    return this.run(() -> {
      System.out.println("Command: setOuttakeVoltagesArcadeCommand 2: Running.");
      double score = drop.getAsDouble();
      this.setVoltages(score * 12);
  }); 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}