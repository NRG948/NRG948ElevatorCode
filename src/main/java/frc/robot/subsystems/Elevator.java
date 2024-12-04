// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  
  //physical parameters of the elevator
  private static final double GEAR_RATIO = 9.0; 
  private static final double SPROCKET_DIAMETER = 0.3; //meters
  private static final double MASS = 2.0; //kilograms
  private static final double METERS_PER_REVOLUTION = (SPROCKET_DIAMETER * Math.PI) / GEAR_RATIO; 

  //trapezoid profile values
  private static final DCMotor MOTOR_PARAMS = DCMotor.getKrakenX60(1);
  private static final double MAX_SPEED = (MOTOR_PARAMS.freeSpeedRadPerSec * SPROCKET_DIAMETER) / GEAR_RATIO; //m/s
  private static final double MAX_ACCELERATION = (2 * MOTOR_PARAMS.stallTorqueNewtonMeters * GEAR_RATIO) / (SPROCKET_DIAMETER * MASS); //m/s^2
  private static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCELERATION);

  //feedforward constants
  private static final double KS = 0.15;
  private static final double KV = 12 / MAX_SPEED;
  private static final double KA = 12 / MAX_ACCELERATION;
  private static final double KG = 9.81 * KA;

  private CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder();

  private final ElevatorFeedforward feedForward = new ElevatorFeedforward(KS, KG, KV, KA);
  private final TrapezoidProfile profile = new TrapezoidProfile(CONSTRAINTS);
  private final Timer timer = new Timer();

  private boolean isSeekingGoal;
  private final TrapezoidProfile.State currentState = new TrapezoidProfile.State();
  private final TrapezoidProfile.State goalState = new TrapezoidProfile.State();

  /** Creates a new Elevator. */
  public Elevator() {
    motor.setIdleMode(IdleMode.kBrake);
    encoder.setPosition(0);
    encoder.setPositionConversionFactor(METERS_PER_REVOLUTION);
    encoder.setVelocityConversionFactor(METERS_PER_REVOLUTION);
    updateSensorState();
  }
  
  public void disable() {
    motor.disable();
    isSeekingGoal = false;
    timer.stop();
  }

  public void setGoalPosition(double pos) {
    timer.reset();
    timer.start();
    isSeekingGoal = true;
    goalState.position = pos;
    goalState.velocity = 0;

  }

  private void updateSensorState() {
    currentState.position = encoder.getPosition();
    currentState.velocity = encoder.getVelocity();
  }

  @Override
  public void periodic() {
    updateSensorState();
    if (isSeekingGoal) {
      TrapezoidProfile.State desiredState = profile.calculate(timer.get(), currentState, goalState);
      double voltage = feedForward.calculate(currentState.velocity, desiredState.velocity, 0.020);
      motor.setVoltage(voltage);
    }
    // This method will be called once per scheduler run
  }
}
