// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

public class Arm extends SubsystemBase {
  //One Motor and Encoder for base of arm
  CANSparkMax m_armMotor = new CANSparkMax(4, MotorType.kBrushless);
  RelativeEncoder m_armMotorEncoder = m_armMotor.getAlternateEncoder(Type.kQuadrature, 8192);

  public Arm() {}

  //Double for position of the motor
  public double armMotorPosition = m_armMotorEncoder.getPosition()*m_armMotorEncoder.getPositionConversionFactor();
  //Unit for forward limit arm motor 180
  public float armForwardLimit = 180;
  //Unit for backward limit arm motor
  public float armReverseLimit = 0;

  public void setArmLimit(){
    m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_armMotor.setSoftLimit(SoftLimitDirection.kForward, armForwardLimit);
    m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse,true);
    m_armMotor.setSoftLimit(SoftLimitDirection.kReverse, armReverseLimit);
  }
  public double getArmDegree(){
    return armMotorPosition*360;
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
    SmartDashboard.putNumber("Arm Degree", getArmDegree());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
