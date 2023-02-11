// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;


public class Drivetrain extends SubsystemBase {

  //NEO Motors
  CANSparkMax m_leftMaster = new CANSparkMax(0,MotorType.kBrushless);
  CANSparkMax m_leftMotor1 = new CANSparkMax(1,MotorType.kBrushless);
  CANSparkMax m_rightMaster = new CANSparkMax(2,MotorType.kBrushless);
  CANSparkMax m_rightMotor1 = new CANSparkMax(3,MotorType.kBrushless);
  MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftMaster, m_leftMotor1);
  MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightMaster, m_rightMotor1);

  //Differential drive tool
  DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftGroup,m_rightGroup);
  //GYROSCOPE made here
  AHRS m_gyroscope = new AHRS(Port.kMXP);
  //used PID values from past HW
  PIDController m_pidController = new PIDController(1,0.1,0.1);



  public Drivetrain() {
     m_leftGroup.setInverted(true);
  }
  public void arcadeDrive(double xSpeed, double zRotation){
    m_differentialDrive.arcadeDrive(0, 0);
  }
  //angle PID with an input parameter
  public void anglePID(double angle){
    //need to turn negative??
    double kPidAdjust = m_pidController.calculate(m_gyroscope.getAngle(),angle);
    m_differentialDrive.arcadeDrive(0,kPidAdjust);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
