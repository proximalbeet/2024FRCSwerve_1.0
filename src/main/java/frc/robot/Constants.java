// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
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
     //public static MedianPIDSource DRIVE_DISTANCE_ENCODERS;
 
     public static Encoder LEFT_FRONT_DRIVE_DIRECTION_ENCODER;
     public static Encoder LEFT_BACK_DRIVE_DIRECTION_ENCODER;
     public static Encoder RIGHT_FRONT_DRIVE_DIRECTION_ENCODER;
     public static Encoder RIGHT_BACK_DRIVE_DIRECTION_ENCODER;
 
     // Direction encoder wrapper that scales to degrees
     //public static PIDSourceExtended LEFT_FRONT_DRIVE_DIRECTION_SCALED;
     //public static PIDSourceExtended LEFT_BACK_DRIVE_DIRECTION_SCALED;
     //public static PIDSourceExtended RIGHT_FRONT_DRIVE_DIRECTION_SCALED;
     //public static PIDSourceExtended RIGHT_BACK_DRIVE_DIRECTION_SCALED;
 
     // Gyro
     //Pigeon2 _pigeon = new Pigeon2(0, "rio");

}
