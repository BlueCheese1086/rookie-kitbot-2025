// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain;

/** Add your docs here. */
public abstract class DrivetrainIO {
    public abstract void setVoltagesArcadeCommand(double left, double right);
    public abstract void setVoltages(double left, double right);
}
