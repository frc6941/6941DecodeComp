package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.LockHeadingCommand;
import org.firstinspires.ftc.teamcode.Commands.PreShootCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;


@TeleOp(name = "TeleOp Drive (Command)", group = "Competition")
public class CommandTeleOp extends CommandOpMode {

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private FeederSubsystem feeder;
    private GamepadEx driverRC;
    private LimelightSubsystem limelight;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        feeder = new FeederSubsystem(hardwareMap);
        driverRC = new GamepadEx(gamepad1);
        limelight = new LimelightSubsystem(hardwareMap);

        new GamepadButton(driverRC, GamepadKeys.Button.START)
                .whenPressed(drive::resetHeading);

        final Trigger leftTrigger = new Trigger(
                () -> driverRC.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
        );
        final Trigger rightTrigger = new Trigger(
                () -> driverRC.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
        );

        leftTrigger.whileActiveContinuous(
                //new ParallelCommandGroup(
                //        new PreShootCommand(shooter),
                        new LockHeadingCommand(
                                drive,
                                driverRC::getLeftX,
                                driverRC::getLeftY,
                                () -> 45 + 90,
                                2.0,
                                telemetry
                        )
                //)
        );

        rightTrigger.whileActiveContinuous(
                new ShootCommand(shooter, feeder)
        );
        new GamepadButton(driverRC, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new IntakeCommand(feeder));

        register(drive);
        register(shooter);
        register(feeder);
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.drive(
                                driverRC.getLeftX(),
                                driverRC.getLeftY(),
                                -driverRC.getRightX()
                        ),
                        drive
                )
        );
        shooter.setDefaultCommand(
                new RunCommand(
                        () -> {
                            shooter.setOpenLoop(0.3);
                            shooter.closeLatch();
                        },
                        shooter
                )

        );
        feeder.setDefaultCommand(
                new RunCommand(
                        () -> feeder.stop(),
                        feeder
                )
        );

    }

    @Override
    public void run() {
        super.run();
        driverRC.readButtons();

        // Update LL with current gyro, then fuse vision if valid
        limelight.updateWithImuHeading(drive.getHeadingDegrees());
        limelight.getPoseEstimate().ifPresent(visionPose -> {
            final double distanceMeters = limelight.getDistanceMeters().orElse(Double.NaN);
            if (!Double.isNaN(distanceMeters)) {
                Pose2d fused = drive.fuseVisionPose(visionPose, distanceMeters);
                telemetry.addData("Vision Fused", true);
                telemetry.addData("Vision Gain", "auto");
            }
        });

        Pose2d pose = drive.getPose();
        Pose2d ghost = limelight.getPoseEstimate().orElse(null);
        display.sendPoseWithGhost(pose,ghost);
        telemetry.update();
    }
}

