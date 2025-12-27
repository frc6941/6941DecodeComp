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

    // 注意：DriveSubsystem 的手柄语义是：leftX=右+、leftY=前+、rightX=逆时针+
    // 而 PinPoint 里程计坐标约定是：x=左+、y=前+（见 DriveSubsystem 顶部注释）
    // 所以这里用两个平移 PID（X=左右、Y=前后）并显式处理符号，避免坐标系混用导致控制方向错误。
    private final PIDController strafeController;
    private final PIDController forwardController;
    private final PIDController headingController;

    private double headingToleranceDeg;
    private double maxTranslationOutput;
    private double maxTurnOutput;

    private double savedDriverInputOffsetDeg = 0.0;
    private boolean savedFieldCentricEnabled = true;

    public GoToPoseCommand(final DriveSubsystem drive, final Pose2d targetPose, final double toleranceIn) {
        this.drive = drive;
        this.targetPose = targetPose;
        this.toleranceIn = toleranceIn;

        strafeController = new PIDController(
                Constants.Drive.GOTO_POSE_X_kP,
                Constants.Drive.GOTO_POSE_X_kI,
                Constants.Drive.GOTO_POSE_X_kD
        );
        forwardController = new PIDController(
                Constants.Drive.GOTO_POSE_Y_kP,
                Constants.Drive.GOTO_POSE_Y_kI,
                Constants.Drive.GOTO_POSE_Y_kD
        );
        headingController = new PIDController(
                Constants.Drive.GOTO_POSE_TURN_kP,
                Constants.Drive.GOTO_POSE_TURN_kI,
                Constants.Drive.GOTO_POSE_TURN_kD
        );

        headingToleranceDeg = Constants.Drive.GOTO_POSE_HEADING_TOLERANCE_DEG;
        maxTranslationOutput = Constants.Drive.GOTO_POSE_MAX_TRANSLATION_OUTPUT;
        maxTurnOutput = Constants.Drive.GOTO_POSE_MAX_TURN_OUTPUT;

        strafeController.setSetPoint(0.0);
        forwardController.setSetPoint(0.0);
        headingController.setSetPoint(0.0);

        strafeController.setTolerance(toleranceIn);
        forwardController.setTolerance(toleranceIn);

        addRequirements(drive);
    }

    public void setTranslationPID(final double kP, final double kI, final double kD) {
        strafeController.setPID(kP, kI, kD);
        forwardController.setPID(kP, kI, kD);
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
        savedFieldCentricEnabled = drive.isFieldCentricEnabled();
        drive.setDriverInputOffsetDeg(0.0);
        // GoToPose 使用 field-centric：输入用“场地坐标系”的 X/Y（经符号适配后符合手柄语义），由 DriveSubsystem 负责转换到机器人坐标。
        drive.setFieldCentricEnabled(true);
        display.setTargetPose(targetPose);
        strafeController.reset();
        forwardController.reset();
        headingController.reset();
        strafeController.setSetPoint(0.0);
        forwardController.setSetPoint(0.0);
        headingController.setSetPoint(0.0);
    }

    @Override
    public void execute() {
        strafeController.setPID(
                GoToPoseTuning.TRANSLATION_kP,
                GoToPoseTuning.TRANSLATION_kI,
                GoToPoseTuning.TRANSLATION_kD
        );
        forwardController.setPID(
                GoToPoseTuning.TRANSLATION_kP,
                GoToPoseTuning.TRANSLATION_kI,
                GoToPoseTuning.TRANSLATION_kD
        );
        headingController.setPID(GoToPoseTuning.TURN_kP, GoToPoseTuning.TURN_kI, GoToPoseTuning.TURN_kD);
        headingToleranceDeg = GoToPoseTuning.HEADING_TOLERANCE_DEG;
        maxTranslationOutput = GoToPoseTuning.MAX_TRANSLATION_OUTPUT;
        maxTurnOutput = GoToPoseTuning.MAX_TURN_OUTPUT;
        headingController.setTolerance(headingToleranceDeg);

        final Pose2d pose = drive.getPose();
        // pose / target 的坐标：y=左+，x=前+（in）
        final double dxLeftIn = targetPose.getY() - pose.getY();
        final double dyFwdIn = targetPose.getX() - pose.getX();
        // DriveSubsystem.drive 的输入：leftX=右+、leftY=前+
        double strafeOut = strafeController.calculate(-dxLeftIn, 0.0);     // dxLeft>0 => 输出<0 => 往左走（正确）
        double forwardOut = forwardController.calculate(dyFwdIn, 0.0);   // dyFwd>0 => 先取负，再 PID => 输出>0 => 往前走（正确）

        strafeOut = clamp(strafeOut, -maxTranslationOutput, maxTranslationOutput);
        forwardOut = clamp(forwardOut, -maxTranslationOutput, maxTranslationOutput);

        // 对角线合成限幅：保持方向但不让合速度超过 maxTranslationOutput
        final double mag = Math.hypot(strafeOut, forwardOut);
        if (mag > maxTranslationOutput && mag > 1e-9) {
            final double s = maxTranslationOutput / mag;
            strafeOut *= s;
            forwardOut *= s;
        }

        // 朝向控制：使用 DriveSubsystem.getHeadingDegrees() 的参考系（与 field-centric 和 LockHeadingCommand 保持一致）
        final double targetHeadingDeg = Math.toDegrees(targetPose.getHeading());
        final double currentHeadingDeg = Math.toDegrees(drive.getPose().getHeading());
        final double headingErrorDeg = shortestDelta360(targetHeadingDeg, currentHeadingDeg);
        double turnOut = headingController.calculate(-headingErrorDeg);
        turnOut = clamp(turnOut, -maxTurnOutput, maxTurnOutput);

        drive.drive(-strafeOut, -forwardOut, turnOut);
    }

    @Override
    public void end(final boolean interrupted) {
        drive.stop();
        drive.setDriverInputOffsetDeg(savedDriverInputOffsetDeg);
        drive.setFieldCentricEnabled(savedFieldCentricEnabled);
        display.clearTargetPose();
    }

    @Override
    public boolean isFinished() {
        final Pose2d pose = drive.getPose();
        final double dxLeftIn = targetPose.getX() - pose.getX();
        final double dyFwdIn = targetPose.getY() - pose.getY();
        final double distance = Math.hypot(dxLeftIn, dyFwdIn);

        final double targetHeadingDeg = Math.toDegrees(targetPose.getHeading());
        final double currentHeadingDeg = Math.toDegrees(drive.getPose().getHeading());
        final double headingErrorDeg = shortestDelta360(targetHeadingDeg, currentHeadingDeg);

        return distance <= toleranceIn && Math.abs(headingErrorDeg) <= headingToleranceDeg;
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
