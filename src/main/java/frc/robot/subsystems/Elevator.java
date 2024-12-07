// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  //feedback constants
  private static final double KP = 1.0;
  private static final double KI = 0;
  private static final double KD = 0;

  private CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder();
  private SparkLimitSwitch upperLimit = motor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
  private SparkLimitSwitch lowerLimit = motor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

  private ElevatorSim simElevator = new ElevatorSim(MOTOR_PARAMS, GEAR_RATIO, MASS, SPROCKET_DIAMETER/2, 0, 1, true, 0);

  private Mechanism2d mechanism2d = new Mechanism2d(0.5, 1.0);
  private MechanismRoot2d mechanismRoot2d = mechanism2d.getRoot("Elevator Root", 0, 0);
  private MechanismLigament2d elevatorMech2d = mechanismRoot2d.append(new MechanismLigament2d("Elevator", 0, 90));

  private final ElevatorFeedforward feedForward = new ElevatorFeedforward(KS, KG, KV, KA);
  private final TrapezoidProfile profile = new TrapezoidProfile(CONSTRAINTS);
  private final Timer timer = new Timer();

  private final ProfiledPIDController feedBack = new ProfiledPIDController(KP, KI, KD, CONSTRAINTS);

  private boolean isSeekingGoal;
  private final TrapezoidProfile.State currentState = new TrapezoidProfile.State();
  private final TrapezoidProfile.State goalState = new TrapezoidProfile.State();
  private boolean atUpperLimit;
  private boolean atLowerLimit;
  private double currentVoltage;

  /** Creates a new Elevator. */
  public Elevator() {
    motor.setIdleMode(IdleMode.kBrake);
    encoder.setPosition(0);
    encoder.setPositionConversionFactor(METERS_PER_REVOLUTION);
    encoder.setVelocityConversionFactor(METERS_PER_REVOLUTION);
    updateSensorState();
    SmartDashboard.putData("Elevator Sim", mechanism2d);
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
    if (RobotBase.isReal()){
      currentState.position = encoder.getPosition();
      currentState.velocity = encoder.getVelocity();
    } else {
      currentState.position = simElevator.getPositionMeters();
      currentState.velocity = simElevator.getVelocityMetersPerSecond();
    }
    atUpperLimit = upperLimit.isPressed();
    atLowerLimit = lowerLimit.isPressed();
    elevatorMech2d.setLength(currentState.position);
  }

  @Override
  public void periodic() {
    updateSensorState();
    if (isSeekingGoal) {
      TrapezoidProfile.State desiredState = profile.calculate(timer.get(), currentState, goalState);
      currentVoltage = feedForward.calculate(currentState.velocity, desiredState.velocity, 0.020);
      currentVoltage += feedBack.calculate(currentState.position, desiredState);
      if ((currentVoltage > 0 && atUpperLimit)) {
        currentVoltage = KG;
      }
      if ((currentVoltage < 0 && atLowerLimit)) {
        currentVoltage = 0;
      }
      motor.setVoltage(currentVoltage);
    }
  }
  @Override
  public void simulationPeriodic(){
    simElevator.setInput(currentVoltage);
    simElevator.update(0.020);
  }
}
