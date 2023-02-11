// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

public class Arm extends SubsystemBase {
  //One Motor and Encoder for base of arm
  CANSparkMax m_armBigMotor = new CANSparkMax(4, MotorType.kBrushless);
  RelativeEncoder m_armBigMotorEncoder = m_armBigMotor.getAlternateEncoder(Type.kQuadrature, 8192);
  //Motor and encoder for the arm extension part
  CANSparkMax m_armSmallMotor = new CANSparkMax(5,MotorType.kBrushless);
  RelativeEncoder m_armSmallMotorEncoder = m_armSmallMotor.getAlternateEncoder(Type.kQuadrature,8192);
  //PID CONTROLLER OBJECt
  PIDController m_PID = new PIDController(1,0.1,0.1);
  public Arm() {}

  //Double for position of the motor
  //Placeholder to build code on, PositionConversion factor is unknown to get degrees from encoder rotation
  public double armBigAnglePosition = m_armBigMotorEncoder.getPosition()*m_armBigMotorEncoder.getPositionConversionFactor();
  //Unit for forward limit arm motor 180 (what units to use?)
  public float armForwardLimit = 90/(float)m_armBigMotorEncoder.getPositionConversionFactor();
  //Unit for backward limit arm motor
  public float armReverseLimit = 0;
  //Placeholder to build code on, PositionConversion factor is unknown to get degrees from encoder rotation
  public double armSmallMotorPosition = m_armSmallMotorEncoder.getPosition()*m_armSmallMotorEncoder.getPositionConversionFactor();
  //r1 is Set arm length
  int r1 = 10101010;
  //r2 is the extension
  //r3 is the total added

  public void setBigArmLimit(){
    m_armBigMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_armBigMotor.setSoftLimit(SoftLimitDirection.kForward, armForwardLimit);
    m_armBigMotor.enableSoftLimit(SoftLimitDirection.kReverse,true);
    m_armBigMotor.setSoftLimit(SoftLimitDirection.kReverse, armReverseLimit);
  }
  public double getArmDegree(){
    return armBigAnglePosition;
  }
  public void setAngleBigMotor(double setDegree){
    //This is 
    double setDegreeConvertedToEncoder = setDegree/m_armBigMotorEncoder.getPositionConversionFactor();
    //PID for the motor to adjust from getArmDegree to setDegree argument
    //GetArmDegree(degrees) = EncoderPosition(encoder)*ConversionFactor(constant) 
    //SetDegreeConverted (encoder)= setDegree(degrees)/ConversionFactor(constant)
    double pidArmAngle = m_PID.calculate(m_armBigMotorEncoder.getPosition(),setDegreeConvertedToEncoder);
    while(armBigAnglePosition!= setDegree){
      m_armBigMotor.set(pidArmAngle);
    }
    m_armBigMotor.set(0);
    }
  //Method use controller for main arm rotation
  public void controllerArmBig(){
    m_armBigMotor.set(RobotContainer.m_armController.getLeftY());
  }
  //next STEP do move smallArm and get the rotation by using encoder
  public void controllerArmSmall(){
    m_armSmallMotor.set(RobotContainer.m_armController.getRightY());
  }
  public void setPoint(int xPoint, int yPoint){
    //distance is the total length minus the hardstuck length of r1
    double distance = Math.sqrt(Math.pow(xPoint,2)+Math.pow(yPoint,2)) - r1;
    double setAngle = Math.atan(yPoint/xPoint);
    //uses previously made method to move big arm angle into setAngle just calculate from setPoint method
    setAngleBigMotor(setAngle);
    double pidSmallArm = m_PID.calculate(armSmallMotorPosition,distance);
    while(armSmallMotorPosition!=distance){
      m_armSmallMotor.set(pidSmallArm);
    }
    m_armSmallMotor.set(0);
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
    getArmDegree();
    
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
