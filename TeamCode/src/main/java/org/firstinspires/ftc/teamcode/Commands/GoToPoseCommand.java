package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.display;
import org.firstinspires.ftc.teamcode.tuning.GoToPoseTuning;

public class GoToPoseCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final Pose2d targetPose;
    private final double toleranceIn;

    private final PIDController translationController;
    private final PIDController headingController;

    private double headingToleranceDeg;
    private double maxTranslationOutput;
    private double maxTurnOutput;
    private double rotationAdjustmentMaxDeg;

    private double savedDriverInputOffsetDeg = 0.0;
    private double lastHeadingErrorDeg = 0.0;

    public GoToPoseCommand(final DriveSubsystem drive, final Pose2d targetPose, final double toleranceIn) {
        this.drive = drive;
        this.targetPose = targetPose;
        this.toleranceIn = toleranceIn;

        translationController = new PIDController(
                Constants.Drive.GOTO_POSE_X_kP,
                Constants.Drive.GOTO_POSE_X_kI,
                Constants.Drive.GOTO_POSE_X_kD
        );
        headingController = new PIDController(
                Constants.Drive.GOTO_POSE_TURN_kP,
                Constants.Drive.GOTO_POSE_TURN_kI,
                Constants.Drive.GOTO_POSE_TURN_kD
        );

        headingToleranceDeg = Constants.Drive.GOTO_POSE_HEADING_TOLERANCE_DEG;
        maxTranslationOutput = Constants.Drive.GOTO_POSE_MAX_TRANSLATION_OUTPUT;
        maxTurnOutput = Constants.Drive.GOTO_POSE_MAX_TURN_OUTPUT;
        rotationAdjustmentMaxDeg = 0.0;

        translationController.setSetPoint(0.0);
        headingController.setSetPoint(0.0);

        translationController.setTolerance(toleranceIn);

        addRequirements(drive);
    }

    public void setTranslationPID(final double kP, final double kI, final double kD) {
        translationController.setPID(kP, kI, kD);
    }

    public void setTurnPID(final double kP, final double kI, final double kD) {
        headingController.setPID(kP, kI, kD);
    }

    public void setHeadingToleranceDeg(final double toleranceDeg) {
        headingController.setTolerance(toleranceDeg);
        headingToleranceDeg = toleranceDeg;
    }

    @Override
    public void initialize() {
        savedDriverInputOffsetDeg = drive.getDriverInputOffsetDeg();
        drive.setDriverInputOffsetDeg(0.0);
        display.setTargetPose(targetPose);
        translationController.reset();
        headingController.reset();
        translationController.setSetPoint(0.0);
        headingController.setSetPoint(0.0);
        lastHeadingErrorDeg = 0.0;
    }

    @Override
    public void execute() {
        translationController.setPID(
                GoToPoseTuning.TRANSLATION_kP,
                GoToPoseTuning.TRANSLATION_kI,
                GoToPoseTuning.TRANSLATION_kD
        );
        headingController.setPID(GoToPoseTuning.TURN_kP, GoToPoseTuning.TURN_kI, GoToPoseTuning.TURN_kD);
        headingToleranceDeg = GoToPoseTuning.HEADING_TOLERANCE_DEG;
        maxTranslationOutput = GoToPoseTuning.MAX_TRANSLATION_OUTPUT;
        maxTurnOutput = GoToPoseTuning.MAX_TURN_OUTPUT;
        rotationAdjustmentMaxDeg = GoToPoseTuning.ROTATION_ADJUSTMENT_MAX_DEG;
        headingController.setTolerance(headingToleranceDeg);

        final Pose2d pose = drive.getPose();
        final double dx = targetPose.getX() - pose.getX();
        final double dy = targetPose.getY() - pose.getY();

        final double headingDegForTransform = drive.getHeadingDegrees();
        final double headingRad = Math.toRadians(headingDegForTransform);
        final double cos = Math.cos(headingRad);
        final double sin = Math.sin(headingRad);

        final double xErrRobot = dx * cos + dy * sin;
        final double yErrRobot = -dx * sin + dy * cos;

        final double distErr = Math.hypot(xErrRobot, yErrRobot);
        double vNorm = -translationController.calculate(distErr, 0.0);
        vNorm = clamp(vNorm, 0.0, maxTranslationOutput);

        final double dirRobotRad = Math.atan2(yErrRobot, xErrRobot);
        final double vxRobot = vNorm * Math.cos(dirRobotRad);
        final double vyRobot = vNorm * Math.sin(dirRobotRad);

        final double vxField = vxRobot * cos - vyRobot * sin;
        final double vyField = vxRobot * sin + vyRobot * cos;

        final double targetHeadingDeg = Math.toDegrees(targetPose.getHeading());
        final double currentHeadingDeg = headingDegForTransform;
        final double headingErrorDeg = shortestDelta360(targetHeadingDeg, currentHeadingDeg);
        final double dirRobotDeg = Math.toDegrees(dirRobotRad);
        final double adjustedHeadingErrorDeg = clamp(
                dirRobotDeg,
                headingErrorDeg - Math.abs(rotationAdjustmentMaxDeg),
                headingErrorDeg + Math.abs(rotationAdjustmentMaxDeg)
        );
        lastHeadingErrorDeg = headingErrorDeg;
        double turnOut = headingController.calculate(-adjustedHeadingErrorDeg);
        turnOut = clamp(turnOut, -maxTurnOutput, maxTurnOutput);

        drive.drive(vxField, vyField, turnOut);
    }

    @Override
    public void end(final boolean interrupted) {
        drive.stop();
        drive.setDriverInputOffsetDeg(savedDriverInputOffsetDeg);
        display.clearTargetPose();
    }

    @Override
    public boolean isFinished() {
        final Pose2d pose = drive.getPose();
        final double dx = targetPose.getX() - pose.getX();
        final double dy = targetPose.getY() - pose.getY();
        final double distance = Math.hypot(dx, dy);
        return distance <= toleranceIn && Math.abs(lastHeadingErrorDeg) <= headingToleranceDeg;
    }

    private double shortestDelta360(final double targetDeg, final double currentDeg) {
        double error = targetDeg - currentDeg;
        error = error % 360.0;
        if (error > 180.0) {
            error -= 360.0;
        } else if (error < -180.0) {
            error += 360.0;
        }
        return error;
    }

    private double clamp(final double value, final double min, final double max) {
        if (value < min) {
            return min;
        }
        if (value > max) {
            return max;
        }
        return value;
    }

}
