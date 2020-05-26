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
    public static final int kFalconFXUnitsPerRevolution = 2048; /* this is constant for Talon FX */


    public static final class BlinkinLEDSubsystemConstants {
        public static final int PWM_PORT = 3;
    }

    public static final class IntakeSubsystemConstants {
        public static final int INTAKE_MOTOR_CAN_ID = 4;
        public static final double MAX_INTAKE_MOTOR_SPEED = 0.5;
        public static final int SOLENOID_CHANNEL = 4;
    }

    public static final class MagazineSubsystemConstants {
        public static final int MAGAZINE_ROTATOR_CAN_ID = 3;
    }
    
    public static final class ControlPanelRotatorSubsystemConstants {
        public static final int CONTROL_PANEL_SOLENOID_ID = 0;
        public static final int CONTROL_PANEL_MANIPULATOR_ID = 2;
    }

    public static final class LiftSubsystemConstants {
        public static final int SOLENIOD_FORWARD_CHANNEL = 5;
        public static final int SOLENIOD_REVERSE_CHANNEL = 6;
    }

    public static final class TankDrivetrainSubsystemConstants {

    }

    public static final class TurretSubsystemConstants {
        public static final int SHOOTER_MOTOR_CAN_ID = 5;
        public static final int FOLLOWER_SHOOTER_MOTOR_CAN_ID = 6;
        public static final int TURRET_ROTATOR_MOTOR_CAN_ID = 7;
        public static final int TURRET_FEEDER_MOTOR_CAN_ID = 1;
        public static final double SHOOTER_MAX_SPEED = 1.0;
        public static final double SHOOTER_DEFAULT_SPEED = 1.0;
        public static final double TURRET_ROTATOR_DEFAULT_SPEED = 0.20; //0.22
        public static final double TURRET_ROTATOR_MAX_SPEED = 0.20;//0.22;
        public static final double kP = -0.1f; //old
        public static final double min_command = 0.05;
        public static final double TURRET_FEEDER_MOTOR_DEFAULT_SPEED = 1.0;//0.5;//1.0;//0.5;//1.0;
        public static final String SHOOTER_SPEED_KEY_STRING = "Shooter Speed";
        public static final String ROTATOR_SPEED_KEY_STRING = "Turret Rotator Speed";
        public static final double SHOOTER_OPERATING_RPM = 6100;
        public static final double HOOD_MAX_EXTENSION = 30; //TODO figure out
        public static final double HOOD_MIN_EXTENSION = 3;
        public static final double HOOD_MAX_SPEED = 1.0;
        public static final double HOOD_DEFAULT_SPEED = HOOD_MAX_SPEED;
        public static final double HOOD_MIN_SPEED = 0.6; //must find
        public static final int HOOD_DEPLOYER_SOLENOID = 7;
    }

    public static final class LimelightSubsystemConstants {
        public static final int POWERPORT_VISION_PIPELINE = 3;
        public static final int LOADING_BAY_VISION_PIPELINE = 1;
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

    public static final class AimtTurretAtPowerPortConstants {
        public static final double kP = 0.0350;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double TARGET_ANGLE = 0.0;
        public static final double TOLERANCE = 2.2; //degrees
    }
}
