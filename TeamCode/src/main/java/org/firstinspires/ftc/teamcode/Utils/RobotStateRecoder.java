package org.firstinspires.ftc.teamcode.Utils;

public class RobotStateRecoder {
    private static DriverAlliance driverAlliance = DriverAlliance.RED;
    private static ShootingPosition shootingPosition = ShootingPosition.CLOSE;

    public static ShootingPosition getShootingPosition() {
        return shootingPosition;
    }

    public static void setShootingPosition(ShootingPosition shootingPosition) {
        RobotStateRecoder.shootingPosition = shootingPosition;
    }

    public static DriverAlliance getDriverAlliance() {
        return driverAlliance;
    }

    public static void setDriverAlliance(DriverAlliance driverAlliance) {
        RobotStateRecoder.driverAlliance = driverAlliance;
    }

    public enum DriverAlliance {
        BLUE,
        RED
    }

    public enum ShootingPosition {
        FAR,
        MIDDLE,
        CLOSE
    }
}
