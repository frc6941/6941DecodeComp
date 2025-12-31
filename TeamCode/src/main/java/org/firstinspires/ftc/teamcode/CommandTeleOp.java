package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.tuning.ShootTuning.TARGET_RPM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CloseShootCommand;
import org.firstinspires.ftc.teamcode.Commands.CloseShootOpenLoopCommand;
import org.firstinspires.ftc.teamcode.Commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.Commands.IndexCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.LockHeadingCommand;
import org.firstinspires.ftc.teamcode.Subsystems.BeamBreakSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Utils.RobotStateRecoder;

@TeleOp(name = "TeleOp Drive (Command)", group = "Competition")
public class CommandTeleOp extends CommandOpMode {

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private FeederSubsystem feeder;
    private GamepadEx driverRC;
    private LimelightSubsystem limelight;

    private BeamBreakSubsystem beamBreak;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        feeder = new FeederSubsystem(hardwareMap);
        driverRC = new GamepadEx(gamepad1);
        limelight = new LimelightSubsystem(hardwareMap);
        beamBreak = new BeamBreakSubsystem(hardwareMap, "shooterBB", 0.5);

        new GamepadButton(driverRC, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() ->
                {
                    RobotStateRecoder.setDriverAlliance(RobotStateRecoder.DriverAlliance.RED);
                    drive.applyDriverAlliance(RobotStateRecoder.getDriverAlliance());
                });
        new GamepadButton(driverRC, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() ->
                {
                    RobotStateRecoder.setDriverAlliance(RobotStateRecoder.DriverAlliance.BLUE);
                    drive.applyDriverAlliance(RobotStateRecoder.getDriverAlliance());
                });

        new GamepadButton(driverRC, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() ->
                {
                    RobotStateRecoder.setDriverAlliance(RobotStateRecoder.DriverAlliance.BLUE);
                    drive.applyDriverAlliance(RobotStateRecoder.getDriverAlliance());
                });

        final Trigger DPAD_DOWN = new Trigger(() -> driverRC.getButton(GamepadKeys.Button.DPAD_DOWN));

        final Trigger leftTrigger = new Trigger(
                () -> driverRC.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
        );
        final Trigger rightTrigger = new Trigger(
                () -> driverRC.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
        );
        final Trigger leftBumper = new Trigger(
                () -> driverRC.getButton(GamepadKeys.Button.LEFT_BUMPER)
        );
        final Trigger rightBumper = new Trigger(
                () -> driverRC.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        );
        final Trigger buttonA = new Trigger(
                () -> driverRC.getButton(GamepadKeys.Button.A)
        );
        final Trigger buttonB = new Trigger(
                () -> driverRC.getButton(GamepadKeys.Button.B)
        );
        final Trigger buttonX = new Trigger(
                () -> driverRC.getButton(GamepadKeys.Button.X)
        );
        final Trigger buttonY = new Trigger(
                () -> driverRC.getButton(GamepadKeys.Button.Y)
        );

        buttonY.whileActiveOnce(
                new InstantCommand(()
                        -> RobotStateRecoder.setShootingPosition(
                        RobotStateRecoder.ShootingPosition.FAR)));
        buttonB.whileActiveOnce(
                new InstantCommand(()
                        -> RobotStateRecoder.setShootingPosition(
                        RobotStateRecoder.ShootingPosition.MIDDLE)));

        buttonA.whileActiveOnce(
                new InstantCommand(()
                        -> RobotStateRecoder.setShootingPosition(
                        RobotStateRecoder.ShootingPosition.CLOSE)));

        rightTrigger.whileActiveOnce(
                new ConditionalCommand(
                        new CloseShootCommand(shooter, feeder, rightBumper, () -> TARGET_RPM),
                        new ConditionalCommand(
                                new CloseShootOpenLoopCommand(shooter, feeder, rightBumper, 1.0),
                                new CloseShootCommand(shooter, feeder, rightBumper, () -> 3200),
                                () -> RobotStateRecoder.getShootingPosition() == RobotStateRecoder.ShootingPosition.FAR
                        ),
                        () -> RobotStateRecoder.getShootingPosition() == RobotStateRecoder.ShootingPosition.CLOSE
                )
        );

//        rightTrigger.whileActiveOnce(
//                new CloseShootCommand(shooter, feeder, rightBumper)
//        );
//        rightTrigger.whileActiveOnce(
//                new CloseShootOpenLoopCommand(shooter, feeder, rightBumper)
//        );
        leftTrigger.whileActiveContinuous(
                new IntakeCommand(feeder)
        );
        leftBumper.whileActiveContinuous(
                new IndexCommand(feeder, -Constants.Feeder.DEFAULT_INTAKE_POWER, -Constants.Feeder.DEFAULT_INDEX_POWER)
        );

        DPAD_DOWN.whileActiveContinuous(
                new IndexCommand(feeder, 0, Constants.Feeder.DEFAULT_INDEX_POWER)
        );

//        rightTrigger.whileActiveContinuous(
//                new LockHeadingCommand(
//                        drive,
//                        driverRC::getLeftX,
//                        driverRC::getLeftY,
//                        () -> LockHeadingTuning.TARGET_DEGREE,
//                        2.0,
//                        telemetry
//                )
//        );

        rightTrigger.whileActiveContinuous(
                new LockHeadingCommand(
                        drive,
                        driverRC::getLeftX,
                        driverRC::getLeftY,
                        () -> {
                            final Pose2d goal = RobotStateRecoder.getDriverAlliance() == RobotStateRecoder.DriverAlliance.BLUE
                                    ? Constants.Field.GOAL_BLUE
                                    : Constants.Field.GOAL_RED;
                            final Pose2d pose = drive.getPose();

                            final double dx = goal.getX() - pose.getX();
                            final double dy = goal.getY() - pose.getY();
                            return Math.toDegrees(Math.atan2(dy, dx));
                        },
                        2.0,
                        telemetry
                )
        );
        register(drive);
        register(shooter);
        register(feeder);
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.drive(
                                driverRC.getLeftX(), // right +
                                driverRC.getLeftY(), // forward +
                                -driverRC.getRightX() // ccw + (left+)
                        ),
                        drive
                )
        );
        shooter.setDefaultCommand(
                new RepeatCommand(
                        new InstantCommand(() -> shooter.setVelocityRpm(3200), shooter))
        );
        feeder.setDefaultCommand(
                new RunCommand(
                        () -> feeder.stop(),
                        feeder
                )
        );

        if (RobotStateRecoder.getDriverAlliance() == RobotStateRecoder.DriverAlliance.RED) {
            buttonX.whileActiveOnce(
                    new ConditionalCommand(
                            new GoToPoseCommand(drive, Constants.Field.GOAL_RED_CLOSE, 0.5),
                            new ConditionalCommand(
                                    new GoToPoseCommand(drive, Constants.Field.GOAL_RED_FAR, 0.5),
                                    new GoToPoseCommand(drive, Constants.Field.GOAL_RED_MIDDLE, 0.5),
                                    () -> RobotStateRecoder.getShootingPosition() == RobotStateRecoder.ShootingPosition.FAR
                            ),
                            () -> RobotStateRecoder.getShootingPosition() == RobotStateRecoder.ShootingPosition.CLOSE
                    )
            );
        } else {
            buttonX.whileActiveOnce(
                    new ConditionalCommand(
                            new GoToPoseCommand(drive, Constants.Field.GOAL_BLUE_CLOSE, 0.5),
                            new ConditionalCommand(
                                    new GoToPoseCommand(drive, Constants.Field.GOAL_BLUE_FAR, 0.5),
                                    new GoToPoseCommand(drive, Constants.Field.GOAL_BLUE_MIDDLE, 0.5),
                                    () -> RobotStateRecoder.getShootingPosition() == RobotStateRecoder.ShootingPosition.FAR
                            ),
                            () -> RobotStateRecoder.getShootingPosition() == RobotStateRecoder.ShootingPosition.CLOSE
                    )
            );
        }
        drive.setFieldCentricEnabled(true);

    }

    @Override
    public void run() {
        super.run();
        driverRC.readButtons();

        // Update LL with current gyro, then fuse vision if valid
        limelight.updateWithImuHeading(drive.getHeadingDegrees() - DriveSubsystem.GYRO_HEADING_OFFSET_DEG);
        limelight.getPoseEstimate().ifPresent(visionPose -> {
            final double distanceMeters = limelight.getDistanceMeters().orElse(Double.NaN);
            if (!Double.isNaN(distanceMeters)) {
                Pose2d fused = drive.fuseVisionPose(visionPose, distanceMeters);
                telemetry.addData("Vision Fused", true);
                telemetry.addData("Vision Gain", "auto");
            }
        });

//        telemetry.addData("LX(right +)", driverRC.getLeftX());
//        telemetry.addData("LY(fwd +)", driverRC.getLeftY());
//        telemetry.addData("RX(ccw +)", -driverRC.getRightX());
        telemetry.addData("Driver Alliance", RobotStateRecoder.getDriverAlliance());
        telemetry.addData("shooting Position", RobotStateRecoder.getShootingPosition());
        telemetry.addData("Driver Input Offset (deg)", drive.getDriverInputOffsetDeg());
        telemetry.addData("Shooter Rpm", shooter.getVelocityRpm());
        telemetry.addData("BB Volt", beamBreak.getVoltage());

        Pose2d pose = drive.getPose();
        Pose2d ghost = limelight.getPoseEstimate().orElse(null);
        display.sendPoseWithGhost(pose, ghost);
        telemetry.update();

    }

}
