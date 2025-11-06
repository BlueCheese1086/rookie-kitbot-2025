package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.RobotMap;

public class DrivetrainIOTalonSRX extends DrivetrainIO {
    TalonSRX leftFrontTalon;
    TalonSRX rightFrontTalon;
    TalonSRX leftBackTalon;
    TalonSRX rightBackTalon;
    TalonSRXConfiguration config;
    VoltageOut leftVoltage;
    VoltageOut rightVoltage;
    
    public DrivetrainIOTalonSRX() {
        leftFrontTalon = new TalonSRX(RobotMap.LEFT_FRONT_TALON_ID);
        rightFrontTalon = new TalonSRX(RobotMap.RIGHT_FRONT_TALON_ID);
        leftBackTalon = new TalonSRX(RobotMap.LEFT_BACK_TALON_ID);
        rightBackTalon = new TalonSRX(RobotMap.RIGHT_BACK_TALON_ID);

        config = new TalonSRXConfiguration();
        leftVoltage = new VoltageOut(0);
        rightVoltage = new VoltageOut(0);

        leftBackTalon.configPeakCurrentLimit(80, 2);
        leftFrontTalon.configPeakCurrentLimit(80, 2);
        rightBackTalon.configPeakCurrentLimit(80, 2);
        rightFrontTalon.configPeakCurrentLimit(80, 2);

        leftBackTalon.follow(leftFrontTalon);
        rightBackTalon.follow(rightFrontTalon);
        
        rightFrontTalon.setInverted(true);
        rightBackTalon.setInverted(true);
    }

    public void setVoltages(double left, double right) {
        leftFrontTalon.set(ControlMode.PercentOutput, left);
        rightFrontTalon.set(ControlMode.PercentOutput, right);
    }
    
}
