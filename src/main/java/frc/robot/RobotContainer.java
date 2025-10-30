package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.Subsystems.OuttakeTest;
import frc.robot.Subsystems.Drivetrain.DrivetrainSubsystem;

/** Add your docs here. */
public class RobotContainer {
      public final static PWMSparkMax m_leftDrive = new PWMSparkMax(0);
        public final static PWMSparkMax m_rightDrive = new PWMSparkMax(1);
                public final static DifferentialDrive m_robotDrive =
                    new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
  public final static XboxController m_controller = new XboxController(0);

  public final static Timer m_timer = new Timer();
    CommandXboxController controller = new CommandXboxController(0);
  
  static DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(); 
  
    static OuttakeTest m_outtakeSubsystem = new OuttakeTest();
    
      public static Command lateralMove(double upDown, double turn, double moveDuration){
            return drivetrainSubsystem.setVoltagesArcadeCommand(
            () -> upDown,
            () -> turn ).withTimeout(moveDuration);
        }
      
        public static Command autonoumousOuttake(double running, double outtakeDuration) {
          return m_outtakeSubsystem.setOuttakeVoltagesArcadeCommand(
          () -> running).withTimeout(outtakeDuration);
      }
      
      private static Command pause(double pauseDuration) {
              return Commands.waitSeconds(pauseDuration);
            }
          
            static Command autonomousCommand = Commands.sequence(
              lateralMove(0.3, 0.0, 0.67),
          pause(0.8),
    autonoumousOuttake(1.0, 0.5),
    lateralMove(-0.4, 0.0, 0.67)/*,
    lateralMove(0.0, 0.5, 0.67),
    lateralMove(0.67, 0.2, 1),
    lateralMove(0, 0.5, 0.4),
    lateralMove(-0.4, 0.0, 0.5),
    pause(3.0),
    lateralMove(0.3, 0.0, 1.5),
    autonoumousOuttake(1.0, 1.0) */
  );

  private double modifyJoystick(double in) {
    if (Math.abs(in) < 0.05) {
      return 0; 
    }
    return in * in * Math.signum(in);
}
    public RobotContainer() {
         
    SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
    System.out.println("Class: Robot Running.");

    drivetrainSubsystem.setDefaultCommand(
      drivetrainSubsystem.setVoltagesArcadeCommand(
          () -> modifyJoystick(-controller.getLeftY()),
          () -> modifyJoystick(-controller.getRightX())));
    
    m_outtakeSubsystem.setDefaultCommand(m_outtakeSubsystem.setVoltagesCommand(()-> 0.0));
     
    controller.rightTrigger().whileTrue(
        m_outtakeSubsystem.setOuttakeVoltagesArcadeCommand(
          () -> controller.getRightTriggerAxis()
        )
      );
    
    controller.leftTrigger().toggleOnTrue(
      drivetrainSubsystem.setVoltagesArcadeCommand(
        () -> modifyJoystick(-controller.getLeftY()* 0.3),
        () -> modifyJoystick(-controller.getRightX() * 0.3))
    );
    }
}