package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class FeederSubsystem extends SubsystemBase {

    private final Motor intakeRoller;
    private final Motor outtakeRoller;

    public FeederSubsystem(final HardwareMap hardwareMap) {
        intakeRoller = buildRollerMotor(
                hardwareMap,
                Constants.Feeder.INTAKE_ROLLER_NAME,
                Constants.Feeder.INTAKE_INVERTED
        );
        outtakeRoller = buildRollerMotor(
                hardwareMap,
                Constants.Feeder.OUTTAKE_ROLLER_NAME,
                Constants.Feeder.OUTTAKE_INVERTED
        );
    }

    public void setIntakeOpenLoop(final double power) {
        intakeRoller.setRunMode(Motor.RunMode.RawPower);
        intakeRoller.set(clamp(power, -1.0, 1.0));
    }

    public void setOuttakeOpenLoop(final double power) {
        outtakeRoller.setRunMode(Motor.RunMode.RawPower);
        outtakeRoller.set(clamp(power, -1.0, 1.0));
    }

    public void setIntakeVelocityRpm(final double rpm) {
        intakeRoller.setRunMode(Motor.RunMode.VelocityControl);
        intakeRoller.set(Math.max(0.0, rpm));
    }

    public void setOuttakeVelocityRpm(final double rpm) {
        outtakeRoller.setRunMode(Motor.RunMode.VelocityControl);
        outtakeRoller.set(Math.max(0.0, rpm));
    }

    public double getIntakeVelocityRpm() {
        return intakeRoller.getCorrectedVelocity();
    }

    public double getOuttakeVelocityRpm() {
        return outtakeRoller.getCorrectedVelocity();
    }

    public void stop() {
        intakeRoller.set(0.0);
        outtakeRoller.set(0.0);
        intakeRoller.setRunMode(Motor.RunMode.RawPower);
        outtakeRoller.setRunMode(Motor.RunMode.RawPower);
    }

    private Motor buildRollerMotor(final HardwareMap hardwareMap,
                                   final String name,
                                   final boolean inverted) {
        final Motor motor = new Motor(hardwareMap, name, Constants.Feeder.ROLLER_GEARING);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setInverted(inverted);
        return motor;
    }

    private double clamp(final double value, final double min, final double max) {
        return Math.max(min, Math.min(max, value));
    }
}

