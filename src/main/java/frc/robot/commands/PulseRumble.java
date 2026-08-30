package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Controller;

public class PulseRumble extends Command {

    private final Controller<?, ?> contr;
    private final double onTime_s, offTime_s;
    private final double strength;

    private double startTime_s;
    private boolean onLast;

    public PulseRumble(Controller<?, ?> _contr, double _onTime_s, double _offTime_s, double _strength) {
        contr = _contr;
        addRequirements(contr);

        onTime_s = _onTime_s;
        offTime_s = _offTime_s;
        strength = _strength;
    }

    @Override
    public void initialize() {
        startTime_s = Timer.getTimestamp();
        onLast = false;
    }

    @Override
    public void execute() {
        double currTime_s = Timer.getTimestamp();

        boolean on = (currTime_s - startTime_s) % (onTime_s + offTime_s) < onTime_s;
        if (on != onLast) {
            if (on) {
                contr.setRumble(strength, strength);
            } else {
                contr.stopRumble();
            }
        }

        onLast = on;
    }

    @Override
    public void end(boolean interrupted) {
        contr.stopRumble();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
