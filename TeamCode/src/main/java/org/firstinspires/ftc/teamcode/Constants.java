package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public final class Constants {

    private Constants() {
    }

    public static final class Drive {
        public static final String FRONT_LEFT_NAME = "LF";
        public static final String FRONT_RIGHT_NAME = "RF";
        public static final String BACK_LEFT_NAME = "LB";
        public static final String BACK_RIGHT_NAME = "RB";
        public static final String IMU_NAME = "IMU";
        public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;
        public static final boolean DEFAULT_FIELD_CENTRIC = true;
        public static final double TURN_kP = 0.01;
        public static final double TURN_kI = 0.0;
        public static final double TURN_kD = 0.0005;
        public static final double TURN_MAX_OUTPUT = 0.6;

        private Drive() {
        }
    }
}

