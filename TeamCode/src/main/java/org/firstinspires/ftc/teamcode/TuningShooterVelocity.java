package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp(name = "Tuning - Shooter Velocity (Dashboard)", group = "Tuning")
public class TuningShooterVelocity extends CommandOpMode {

    private ShooterSubsystem shooter;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new ShooterSubsystem(hardwareMap);
        register(shooter);

        shooter.setDefaultCommand(
                new RunCommand(
                        () -> {
                            shooter.applyVelocityTuning(
                                    Constants.Shooter.VELO_kP,
                                    Constants.Shooter.VELO_kI,
                                    Constants.Shooter.VELO_kD,
                                    Constants.Shooter.FF_kS,
                                    Constants.Shooter.FF_kV,
                                    Constants.Shooter.FF_kA
                            );

                            if (Constants.Shooter.TUNING_ENABLED) {
                                shooter.setVelocityRpm(Constants.Shooter.TUNING_TARGET_RPM);
                            } else {
                                shooter.stop();
                            }
                        },
                        shooter
                )
        );
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("Tuning Enabled", Constants.Shooter.TUNING_ENABLED);
        telemetry.addData("Target RPM", Constants.Shooter.TUNING_TARGET_RPM);
        telemetry.addData("Measured RPM", shooter.getVelocityRpm());
        telemetry.addData("kP/kI/kD", "%.5f / %.5f / %.5f",
                Constants.Shooter.VELO_kP, Constants.Shooter.VELO_kI, Constants.Shooter.VELO_kD);
        telemetry.addData("kS/kV/kA", "%.5f / %.5f / %.5f",
                Constants.Shooter.FF_kS, Constants.Shooter.FF_kV, Constants.Shooter.FF_kA);
        telemetry.update();
    }
}

