package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive drive;
    private final IMU imu;
    private boolean fieldCentricEnabled = Constants.Drive.DEFAULT_FIELD_CENTRIC;

    public DriveSubsystem(final HardwareMap hardwareMap) {
        final Motor frontLeft = buildDriveMotor(hardwareMap, Constants.Drive.FRONT_LEFT_NAME);
        final Motor frontRight = buildDriveMotor(hardwareMap, Constants.Drive.FRONT_RIGHT_NAME);
        final Motor backLeft = buildDriveMotor(hardwareMap, Constants.Drive.BACK_LEFT_NAME);
        final Motor backRight = buildDriveMotor(hardwareMap, Constants.Drive.BACK_RIGHT_NAME);

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        imu = hardwareMap.get(IMU.class, Constants.Drive.IMU_NAME);
        final IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        Constants.Drive.IMU_LOGO_DIRECTION,
                        Constants.Drive.IMU_USB_DIRECTION
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();
    }

    private Motor buildDriveMotor(final HardwareMap hardwareMap, final String name) {
        final Motor motor = new Motor(hardwareMap, name, Motor.GoBILDA.RPM_435);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    public void drive(final double leftX, final double leftY, final double rightX) {
        if (fieldCentricEnabled) {
            drive.driveFieldCentric(leftX, leftY, rightX, getHeadingDegrees(), false);
        } else {
            drive.driveRobotCentric(leftX, leftY, rightX, false);
        }
    }

    public void stop() {
        drive.driveRobotCentric(0.0, 0.0, 0.0, false);
    }

    public double getHeadingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        imu.resetYaw();
    }
}

