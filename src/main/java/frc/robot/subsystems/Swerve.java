// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.sensors.Pigeon2;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PIDOutput;


public class Swerve extends SubsystemBase {  /** Creates a new ExampleSubsystem. */


  // Motors
  private static Spark LEFT_FRONT_DRIVE_SPEED_MOTOR;
  private static Spark LEFT_BACK_DRIVE_SPEED_MOTOR;
  private static Spark RIGHT_FRONT_DRIVE_SPEED_MOTOR;
  private static Spark RIGHT_BACK_DRIVE_SPEED_MOTOR;

  private static Spark LEFT_FRONT_DRIVE_DIRECTION_MOTOR;
  private static Spark LEFT_BACK_DRIVE_DIRECTION_MOTOR;
  private static Spark RIGHT_FRONT_DRIVE_DIRECTION_MOTOR;
  private static Spark RIGHT_BACK_DRIVE_DIRECTION_MOTOR;

  // Encoders
  public static Encoder LEFT_FRONT_DRIVE_DISTANCE_ENCODER;
  public static Encoder LEFT_BACK_DRIVE_DISTANCE_ENCODER;
  public static Encoder RIGHT_FRONT_DRIVE_DISTANCE_ENCODER;
  public static Encoder RIGHT_BACK_DRIVE_DISTANCE_ENCODER;
  public static MedianPIDSource DRIVE_DISTANCE_ENCODERS;

  public static Encoder LEFT_FRONT_DRIVE_DIRECTION_ENCODER;
  public static Encoder LEFT_BACK_DRIVE_DIRECTION_ENCODER;
  public static Encoder RIGHT_FRONT_DRIVE_DIRECTION_ENCODER;
  public static Encoder RIGHT_BACK_DRIVE_DIRECTION_ENCODER;

  // Direction encoder wrapper that scales to degrees
  public static PIDSourceExtended LEFT_FRONT_DRIVE_DIRECTION_SCALED;
  public static PIDSourceExtended LEFT_BACK_DRIVE_DIRECTION_SCALED;
  public static PIDSourceExtended RIGHT_FRONT_DRIVE_DIRECTION_SCALED;
  public static PIDSourceExtended RIGHT_BACK_DRIVE_DIRECTION_SCALED;

  // Gyro
  Pigeon2 _pigeon = new Pigeon2(0, "rio");


  public Swerve() {
      // Motors
      LEFT_FRONT_DRIVE_SPEED_MOTOR = new Spark(RobotMap.LEFT_FRONT_DRIVE_SPEED_MOTOR_PIN);
      LEFT_BACK_DRIVE_SPEED_MOTOR = new Spark(RobotMap.LEFT_BACK_DRIVE_SPEED_MOTOR_PIN);
      RIGHT_FRONT_DRIVE_SPEED_MOTOR = new Spark(RobotMap.RIGHT_FRONT_DRIVE_SPEED_MOTOR_PIN);
      RIGHT_BACK_DRIVE_SPEED_MOTOR = new Spark(RobotMap.RIGHT_BACK_DRIVE_SPEED_MOTOR_PIN);

      LEFT_FRONT_DRIVE_DIRECTION_MOTOR = new Spark(RobotMap.LEFT_FRONT_DRIVE_DIRECTION_MOTOR_PIN);
      LEFT_BACK_DRIVE_DIRECTION_MOTOR = new Spark(RobotMap.LEFT_BACK_DRIVE_DIRECTION_MOTOR_PIN);
      RIGHT_FRONT_DRIVE_DIRECTION_MOTOR = new Spark(RobotMap.RIGHT_FRONT_DRIVE_DIRECTION_MOTOR_PIN);
      RIGHT_BACK_DRIVE_DIRECTION_MOTOR = new Spark(RobotMap.RIGHT_BACK_DRIVE_DIRECTION_MOTOR_PIN);

      // Encoders
      LEFT_FRONT_DRIVE_DISTANCE_ENCODER = new Encoder(RobotMap.LEFT_FRONT_DRIVE_DISTANCE_ENCODER_PIN_A, RobotMap.LEFT_FRONT_DRIVE_DISTANCE_ENCODER_PIN_B);
      LEFT_BACK_DRIVE_DISTANCE_ENCODER = new Encoder(RobotMap.LEFT_BACK_DRIVE_DISTANCE_ENCODER_PIN_A, RobotMap.LEFT_BACK_DRIVE_DISTANCE_ENCODER_PIN_B);
      RIGHT_FRONT_DRIVE_DISTANCE_ENCODER = new Encoder(RobotMap.RIGHT_FRONT_DRIVE_DISTANCE_ENCODER_PIN_A, RobotMap.RIGHT_FRONT_DRIVE_DISTANCE_ENCODER_PIN_B);
      RIGHT_BACK_DRIVE_DISTANCE_ENCODER = new Encoder(RobotMap.RIGHT_BACK_DRIVE_DISTANCE_ENCODER_PIN_A, RobotMap.RIGHT_BACK_DRIVE_DISTANCE_ENCODER_PIN_B);
      DRIVE_ENCODERS = new MedianPIDSource(LEFT_FRONT_DRIVE_DISTANCE_ENCODER, LEFT_BACK_DRIVE_DISTANCE_ENCODER, RIGHT_FRONT_DRIVE_DISTANCE_ENCODER, RIGHT_BACK_DRIVE_DISTANCE_ENCODER);

     Encoder LEFT_FRONT_DRIVE_DIRECTION_ENCODER = new Encoder(RobotMap.LEFT_FRONT_DRIVE_DIRECTION_ENCODER_PIN_A, RobotMap.LEFT_FRONT_DRIVE_DIRECTION_ENCODER_PIN_B);
     Encoder LEFT_BACK_DRIVE_DIRECTION_ENCODER = new Encoder(RobotMap.LEFT_BACK_DRIVE_DIRECTION_ENCODER_PIN_A, RobotMap.LEFT_BACK_DRIVE_DIRECTION_ENCODER_PIN_B);
     Encoder RIGHT_FRONT_DRIVE_DIRECTION_ENCODER = new Encoder(RobotMap.RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_PIN_A, RobotMap.RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_PIN_B);
     Encoder RIGHT_BACK_DRIVE_DIRECTION_ENCODER = new Encoder(RobotMap.RIGHT_BACK_DRIVE_DIRECTION_ENCODER_PIN_A, RobotMap.RIGHT_BACK_DRIVE_DIRECTION_ENCODER_PIN_B);

     // Direction encoder wrapper that scales to degrees
     LEFT_FRONT_DRIVE_DIRECTION_SCALED = new PIDSourceExtended(LEFT_FRONT_DRIVE_DIRECTION_ENCODER);
     LEFT_BACK_DRIVE_DIRECTION_SCALED = new PIDSourceExtended(LEFT_BACK_DRIVE_DIRECTION_ENCODER);
     RIGHT_FRONT_DRIVE_DIRECTION_SCALED = new PIDSourceExtended(RIGHT_FRONT_DRIVE_DIRECTION_ENCODER);
     RIGHT_BACK_DRIVE_DIRECTION_SCALED = new PIDSourceExtended(RIGHT_BACK_DRIVE_DIRECTION_ENCODER);

     // Gyro
     _pigeon = new Pigeon2(RobotMap.MXP_PORT);
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
