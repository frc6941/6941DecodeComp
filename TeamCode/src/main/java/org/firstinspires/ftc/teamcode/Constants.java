package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public final class Constants {

    private Constants() {
    }

    public static final class Drive {

        public static final String FRONT_LEFT_NAME = "LF";
        public static final String FRONT_RIGHT_NAME = "RF";
        public static final String BACK_LEFT_NAME = "LB";
        public static final String BACK_RIGHT_NAME = "RB";
        public static final String ODO_PARALLEL_NAME = "LB";
        public static final String ODO_PERP_NAME = "LF";
        public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION
                = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION
                = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        public static final boolean DEFAULT_FIELD_CENTRIC = true;

        public static final double DRIVER_INPUT_OFFSET_BLUE_DEG = -90.0;
        public static final double DRIVER_INPUT_OFFSET_RED_DEG = 90.0;

        public static final double GOTO_POSE_X_kP = 0.045;
        public static final double GOTO_POSE_X_kI = 0.0;
        public static final double GOTO_POSE_X_kD = 0.0;

        public static final double GOTO_POSE_Y_kP = 0.045;
        public static final double GOTO_POSE_Y_kI = 0.0;
        public static final double GOTO_POSE_Y_kD = 0.0;

        public static final double GOTO_POSE_TURN_kP = 0.05;
        public static final double GOTO_POSE_TURN_kI = 0.0;
        public static final double GOTO_POSE_TURN_kD = 0.0;

        public static final double GOTO_POSE_HEADING_TOLERANCE_DEG = 2.0;
        public static final double GOTO_POSE_MAX_TRANSLATION_OUTPUT = 0.6;
        public static final double GOTO_POSE_MAX_TURN_OUTPUT = 0.6;

        // LockHeading（Dashboard 可通过 LockHeadingTuning 覆盖这些默认值）
        public static final double LOCK_HEADING_TURN_kP = 0.05;
        public static final double LOCK_HEADING_TURN_kI = 0.0;
        public static final double LOCK_HEADING_TURN_kD = 0.0;
        public static final double LOCK_HEADING_MAX_OUTPUT = 0.6;
        public static final double LOCK_HEADING_TARGET_DEG = 0.0;

        private Drive() {
        }
    }

    public static final class Auto {

        /**
         * 只动底盘的“按时间驱动”自动参数（可按需求改）。 注意：这里的坐标约定和 TeleOp 保持一致： - leftX: 右为正 -
         * leftY: 前为正 - rightX: 逆时针为正
         */
        public static final double DRIVE_SECONDS = 2;
        public static final double DRIVE_LEFT_X = 0.0;
        public static final double DRIVE_LEFT_Y = 0.25;
        public static final double DRIVE_RIGHT_X = 0.0;

        private Auto() {
        }
    }

    public static final class NearShootAuto {

        /**
         * 只动底盘的“按时间驱动”自动参数（可按需求改）。 注意：这里的坐标约定和 TeleOp 保持一致： - leftX: 右为正 -
         * leftY: 前为正 - rightX: 逆时针为正
         */
        public static final double DRIVE_SECONDS = 2;
        public static final double DRIVE_LEFT_X = 0.25;
        public static final double DRIVE_LEFT_Y = -0.25;
        public static final double DRIVE_RIGHT_X = 0.0;

        private NearShootAuto() {
        }
    }

    public static final class Shooter {

        public static final String LEADER_NAME = "shooterLeader";
        public static final String FOLLOWER_NAME = "shooterFollower";
        public static final String LATCH_NAME = "latch";
        public static final Motor.GoBILDA GEARING = Motor.GoBILDA.BARE;
        public static final double TICKS_PER_REV_OUTPUT = 28.0;
        public static final double OUTPUT_TO_WHEEL_RATIO = 1.0 / 1.0;
        public static final boolean LEADER_INVERTED = true;
        public static final boolean FOLLOWER_INVERTED = false;
        public static final double LATCH_CLOSED_POS = 0.65;
        public static final double LATCH_OPEN_POS = 0.4;
        public static final double DEFAULT_POWER = 0.6;

        // Shooter Velocity PIDF（Dashboard 可通过 ShooterPidTuning 覆盖这些默认值）
        public static final double TARGET_RPM = 2000.0;
        public static final double VELOCITY_kP = 400.0;
        public static final double VELOCITY_kI = 0.0;
        public static final double VELOCITY_kD = 0.0;
        public static final double VELOCITY_kF = 11.7;
        public static final double MIN_POWER = 0.0;
        public static final double MAX_POWER = 1.0;
        public static final double RPM_TOLERANCE = 200.0;

        private Shooter() {
        }
    }

    public static final class Feeder {

        public static final String INTAKE_ROLLER_NAME = "intakeRoller";
        public static final String INDEX_ROLLER_NAME = "indexRoller";
        public static final Motor.GoBILDA ROLLER_GEARING = Motor.GoBILDA.RPM_1150;
        public static final boolean INTAKE_INVERTED = true;
        public static final boolean INDEX_INVERTED = true;
        public static final double DEFAULT_INTAKE_POWER = 1;

        public static final double SHOOT_INTAKE_POWER = 1.0;
        public static final double DEFAULT_INDEX_POWER = 1.0;//0.8

        private Feeder() {
        }
    }

    public final static class Field {

        public static final Pose2d GOAL_BLUE = new Pose2d(-70, -70, 0);
        public static final Pose2d GOAL_RED = new Pose2d(-70, +70, 0);

        public static final Pose2d GOAL_BLUE_FRONT = new Pose2d(-37, -37, 0);
        public static final Pose2d GOAL_RED_FRONT = new Pose2d(-37, +37, 0);
    }
}
