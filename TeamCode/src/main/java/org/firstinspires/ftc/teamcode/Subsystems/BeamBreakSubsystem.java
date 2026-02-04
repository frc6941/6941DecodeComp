package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BeamBreakSubsystem extends SubsystemBase {

    private final AnalogInput beamBreak;
    private final double defaultToleranceVoltage;

    public BeamBreakSubsystem(HardwareMap hardwareMap, String name, double defaultToleranceVoltage) {
        beamBreak = hardwareMap.get(AnalogInput.class, name);
        this.defaultToleranceVoltage = defaultToleranceVoltage;
    }


    public boolean isBeamBreakOn() {
        return isBeamBreakOn(defaultToleranceVoltage);
    }

    public boolean isBeamBreakOn(double toleranceVoltage) {
        return beamBreak.getVoltage() <= toleranceVoltage;
    }

    public double getVoltage() {
        return beamBreak.getVoltage();
    }
}
