package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;

public class DriveSubsystem extends SubsystemBase {

    // Pinpoint expects offsets in mm: x = sideways (left +), y = forward +
    private static final double PINPOINT_X_OFFSET_MM = -0.05;
    private static final double PINPOINT_Y_OFFSET_MM = -180.0;
    public static final double GYRO_HEADING_OFFSET_DEG = 180;

    private final MecanumDrive drive;
    private final GoBildaPinpointDriver pinpoint;
    private Pose2d rawPose = new Pose2d(0, 0, 0);
    private Pose2d poseOffset = new Pose2d(0, 0, 0);
    private boolean fieldCentricEnabled = Constants.Drive.DEFAULT_FIELD_CENTRIC;

    public DriveSubsystem(final HardwareMap hardwareMap) {
        final Motor frontLeft = buildDriveMotor(hardwareMap, Constants.Drive.FRONT_LEFT_NAME);
        final Motor frontRight = buildDriveMotor(hardwareMap, Constants.Drive.FRONT_RIGHT_NAME);
        final Motor backLeft = buildDriveMotor(hardwareMap, Constants.Drive.BACK_LEFT_NAME);
        final Motor backRight = buildDriveMotor(hardwareMap, Constants.Drive.BACK_RIGHT_NAME);

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "PinPoint");
        pinpoint.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
    }

    private Motor buildDriveMotor(final HardwareMap hardwareMap, final String name) {
        final Motor motor = new Motor(hardwareMap, name, Motor.GoBILDA.RPM_435);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    @Override
    public void periodic() {
        pinpoint.update();
        final Pose2D p = pinpoint.getPosition();
        rawPose = new Pose2d(
                p.getX(DistanceUnit.INCH),
                p.getY(DistanceUnit.INCH),
                p.getHeading(AngleUnit.RADIANS)
        );
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

    public Pose2d getPose() {
        return addOffset(rawPose, poseOffset);
    }

    public void setPose(final Pose2d pose) {
        Pose2d current = getPose();
        double dx = pose.getX() - current.getX();
        double dy = pose.getY() - current.getY();
        poseOffset = new Pose2d(poseOffset.getX() + dx, poseOffset.getY() + dy, 0.0);
    }

    /**
     * Blend the current Pinpoint pose with a vision pose using a distance-based
     * gain, then commit the fused pose. Returns the fused pose.
     */
    public Pose2d fuseVisionPose(final Pose2d visionPose, final double distanceMeters) {
        final Pose2d base = getPose();
        final double gain = computeVisionGain(distanceMeters);
        final double x = lerp(base.getX(), visionPose.getX(), gain);
        final double y = lerp(base.getY(), visionPose.getY(), gain);
        final Pose2d fused = new Pose2d(x, y, base.getHeading()); // keep IMU heading
        setPose(fused);
        return fused;
    }

    public double getHeadingDegrees() {
        double heading = Math.toDegrees(getPose().getHeading());
        heading += GYRO_HEADING_OFFSET_DEG; // 添加 gyro offset 修正旋转中心
        return heading;
    }

    public void resetHeading() {
        pinpoint.resetPosAndIMU();
    }

    private double computeVisionGain(final double distanceMeters) {
        double gain = 0.35 / (distanceMeters + 0.5);
        return clamp(gain, 0.05, 0.6);
    }

    private double lerp(final double a, final double b, final double t) {
        return a + (b - a) * t;
    }

    private double angleWrap(final double angleRad) {
        double wrapped = angleRad;
        while (wrapped > Math.PI) {
            wrapped -= 2.0 * Math.PI;
        }
        while (wrapped < -Math.PI) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped;
    }

    private double clamp(final double value, final double min, final double max) {
        return Math.max(min, Math.min(max, value));
    }

    private Pose2d addOffset(final Pose2d base, final Pose2d offset) {
        return new Pose2d(
                base.getX() + offset.getX(),
                base.getY() + offset.getY(),
                angleWrap(base.getHeading() + offset.getHeading())
        );
    }
}
