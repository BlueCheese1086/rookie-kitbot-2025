package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.RobotMap;


public class DrivetrainIOSparkMax extends DrivetrainIO {
    SparkMax leftFrontSparkMax;
    SparkMax rightFrontSparkMax;
    SparkMax leftBackSparkMax;
    SparkMax rightBackSparkMax;
    SparkMaxConfig config;
    SparkMaxConfig leftFollowConfig;
    SparkMaxConfig rightFollowConfig;
    VoltageOut leftVoltage = new VoltageOut(0);
    VoltageOut rightVoltage = new VoltageOut(0);

    public DrivetrainIOSparkMax() {
        leftFrontSparkMax = new SparkMax(RobotMap.LEFT_FRONT_SPARKMAX_ID, SparkMax.MotorType.kBrushless);
        rightFrontSparkMax = new SparkMax(RobotMap.RIGHT_FRONT_SPARKMAX_ID, SparkMax.MotorType.kBrushless);
        leftBackSparkMax = new SparkMax(RobotMap.LEFT_BACK_SPARKMAX_ID, SparkMax.MotorType.kBrushless);
        rightBackSparkMax = new SparkMax(RobotMap.RIGHT_BACK_SPARKMAX_ID, SparkMax.MotorType.kBrushless);

        config = new SparkMaxConfig();
        leftFollowConfig = new SparkMaxConfig();
        rightFollowConfig = new SparkMaxConfig();
        
        leftFrontSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightFrontSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        config.idleMode(IdleMode.kBrake);
        leftFollowConfig.follow(leftFrontSparkMax);
        rightFollowConfig.follow(rightFrontSparkMax);

        leftBackSparkMax.configure(leftFollowConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightBackSparkMax.configure(rightFollowConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setVoltages(double left, double right) {
        leftFrontSparkMax.set(left);
        rightFrontSparkMax.set(-right);
    }
}
