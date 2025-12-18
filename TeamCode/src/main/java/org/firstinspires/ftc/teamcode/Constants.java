package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
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
        public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;
        public static final boolean DEFAULT_FIELD_CENTRIC = true;

        /**
         * FTC 官方场地坐标系以 Red Wall 为参考：+Y 远离 Red Wall、朝向 Blue Wall。
         * 若驾驶员站在 Blue Wall 一侧，为了让“推前=远离自己”，需要在 field-centric 里额外旋转 180°。
         */
        public enum DriverStationPerspective {
            RED,
            BLUE
        }

        public static DriverStationPerspective DRIVER_STATION_PERSPECTIVE = DriverStationPerspective.RED;
        public static double BLUE_PERSPECTIVE_ROTATION_DEG = 180.0;

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
        public static final boolean LEADER_INVERTED = false;
        public static final boolean FOLLOWER_INVERTED = true;
        public static final double LATCH_CLOSED_POS = 0.4;
        public static final double LATCH_OPEN_POS = 0;

        // FTC Dashboard 远程调参（类似 FRC NetworkTables 的在线参数更新）
        public static boolean TUNING_ENABLED = false;
        public static double TUNING_TARGET_RPM = 0.0;
        public static double VELO_kP = 0.0;
        public static double VELO_kI = 0.0;
        public static double VELO_kD = 0.0;
        public static double FF_kS = 0.0;
        public static double FF_kV = 0.0;
        public static double FF_kA = 0.0;

        private Shooter() {
        }
    }

    public static final class Feeder {
        public static final String INTAKE_ROLLER_NAME = "intakeRoller";
        public static final String OUTTAKE_ROLLER_NAME = "outtakeRoller";
        public static final Motor.GoBILDA ROLLER_GEARING = Motor.GoBILDA.RPM_1150;
        public static final boolean INTAKE_INVERTED = true;
        public static final boolean OUTTAKE_INVERTED = true;

        private Feeder() {
        }
    }
}

