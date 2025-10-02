// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
//import edu.wpi.first.wpilibj.PowerDistribution;
//import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.
import frc.robot.Subsystems.DrivetrainSubsystem;
//import frc.robot.Subsystems.OuttakeSubsystem;
import frc.robot.Subsystems.OuttakeTest;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
  private final XboxController m_controller = new XboxController(0);

  private final Timer m_timer = new Timer();

  CommandXboxController controller = new CommandXboxController(0);
  CommandXboxController controller2 = new CommandXboxController(1);
  
  DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(); 

  OuttakeTest m_outtakeSubsystem = new OuttakeTest();

  private Command lateralMove(double upDown, double turn, double moveDuration){
    return drivetrainSubsystem.setVoltagesArcadeCommand(
      () -> upDown,
      () -> turn).withTimeout(moveDuration);
  }

  private Command autonoumousOuttake(double running, double outtakeDuration) {
    return m_outtakeSubsystem.setOuttakeVoltagesArcadeCommand(
      () -> running).withTimeout(outtakeDuration);
  }
  
  private Command pause(double pauseDuration) {
    return Commands.waitSeconds(pauseDuration);
  }

  private Command autonomousCommand = Commands.sequence(
    // -- Middle to Reef --
    lateralMove(0.3, 0.0, 0.6),
    pause(1.2),
    autonoumousOuttake(1.0, 0.5),
    pause(0.5),
    autonoumousOuttake(0.0, 0.1),
    lateralMove(-0.3,0.0, 0.5),
    pause(1.0),
    lateralMove(0.0, 0.8, 0.4),
    lateralMove(0.4, 0.0, 0.5),
    pause(1.0),
    lateralMove(0.0, -0.9, 0.2),
    lateralMove(0.5,0.0,1.3),
    //full turn?
    lateralMove(0.0,0.8,0.9),
    lateralMove(0.3,0.0,1.5)
    /* 
    autonoumousOuttake(1.0,0.5),
    lateralMove(-0.3, 0.0,1.5),
    pause(3.0),
    lateralMove(0.3, 0.0, 1.5),
    autonoumousOuttake(1.0,0.5 ),
    lateralMove(-0.3, 0.0, 1.5)
    */
  );

  private double modifyJoystick(double in) {
    if (Math.abs(in) < 0.05) {
      return 0; 
    }
    return in * in * Math.signum(in);
  } 


  /** Called once at the beginning of the robot program. */
  public Robot() {
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
     
    controller2.rightTrigger().whileTrue(
        m_outtakeSubsystem.setOuttakeVoltagesArcadeCommand(
          () -> controller2.getRightTriggerAxis()
        )
      );

    controller.x().onTrue(
      lateralMove(0.0, 0.75, 0.5)
    );
    
    controller.leftTrigger().toggleOnTrue(
      drivetrainSubsystem.setVoltagesArcadeCommand(
        () -> modifyJoystick(-controller.getLeftY()* 0.67),
        () -> modifyJoystick(-controller.getRightX() * 0.67))
    );
    
  }





  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
    CommandScheduler.getInstance().schedule(autonomousCommand);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    autonomousCommand.cancel();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();


    

  }
}
