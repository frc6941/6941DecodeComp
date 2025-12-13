package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp Drive (Command)", group = "Competition")
public class CommandTeleOp extends CommandOpMode {

    private DriveSubsystem drive;
    private GamepadEx driverRC;

    @Override
    public void initialize() {
        drive = new DriveSubsystem(hardwareMap);
        driverRC = new GamepadEx(gamepad1);

        new GamepadButton(driverRC, GamepadKeys.Button.START)
                .whenPressed(drive::resetHeading);

        new GamepadButton(driverRC, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new LockHeadingCommand(
                        drive,
                        driverRC::getLeftX,
                        driverRC::getLeftY,
                        () -> 0.0,
                        2.0
                ));

        register(drive);
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.drive(
                                driverRC.getLeftX(),
                                driverRC.getLeftY(),
                                driverRC.getRightX()
                        ),
                        drive
                )
        );
    }

    @Override
    public void run() {
        super.run();
        driverRC.readButtons();
        telemetry.addData("Heading", drive.getHeadingDegrees());
        telemetry.update();
    }
}

