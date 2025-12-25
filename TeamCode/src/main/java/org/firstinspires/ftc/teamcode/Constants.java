package org.firstinspires.ftc.teamcode;

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

        public static final double TURN_kP = 0.05;
        public static final double TURN_kI = 0.0;
        public static final double TURN_kD = 0.000;
        public static final double TURN_MAX_OUTPUT = 0.6;

        private Drive() {
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
        public static final double DEFAULT_INDEX_POWER = 1;

        private Feeder() {
        }
    }
}
