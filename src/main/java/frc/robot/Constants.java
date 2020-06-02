/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class FalconFXConstants {
        public static final int ENCODER_UNITS_PER_REV = 2048; /* this is constant for Talon FX */
        public static final double MOTOR_STALL_TORQUE_Nm = 4.69; //N*m
    }

   


    public static final class BlinkinLEDSubsystemConstants {
        public static final int PWM_PORT = 3;
    }

    public static final class DiplomaArmSubsystemConstants {
        public static final int DIPLOMA_ARM_CAN_ID=7;//TODO arbituary need to find out
        public static final int GEAR_RATIO=0;//TODO find this 
        public static final double ARM_WEIGHT_N=0.0;
        public static final double COM_DISTANCE_m=0.0; //COM=Center of Mass
        public static final int NUMBER_OF_MOTORS=0;
        public static final double ARM_ANGLE_OFFSET_AT_REST=0.0;//TODO find out
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double MAX_VELOCITY= 4.0;
        public static final double MAX_ACCELERATION= 10.0;

    }

    public static final class MoveDiplomaArmConstants{
        public static final double STUDENT_POSITION_DEGREES = 0.0; //TODO Find
        public static final double PRESDIENT_POSITION_DEGREES = 0.0; //TODO find
    }

    public static final class RotateTurretGearConstants{
        public static final double STUDENT_POSITION_DEGREES = 0.0; //TODO Find
        public static final double PRESDIENT_POSITION_DEGREES = 0.0; //TODO find
    }

    public static final class TurretGearPIDSubsystemConstants {
        public static final int TURRET_ROTATOR_MOTOR_CAN_ID = 6;
        public static final double TURRET_ROTATOR_DEFAULT_SPEED = 0.20; //0.22
        public static final double TURRET_ROTATOR_MAX_SPEED = 0.20;//0.22;
        public static final double min_command = 0.05;
        public static final String ROTATOR_SPEED_KEY_STRING = "Turret Rotator Speed";
        public static final double GEAR_RATIO = 0.0; //find
        public static final double kP = 0.0350;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double TOLERANCE = 2.2; //degrees

    }

    public static final class ControllerConstants {
        public static final int BUTTON_BOX_CONTROLLER_PORT = 0;
        public static final int JOYSTICK_CONTROLLER_PORT = 1;
    }

    public static final class MecanumDriveConstants {
        public static final int FRONT_LEFT_MOTOR = 1;
        public static final int REAR_LEFT_MOTOR = 3;
        public static final int FRONT_RIGHT_MOTOR = 2;
        public static final int REAR_RIGHT_MOTOR = 4;
    }

}
