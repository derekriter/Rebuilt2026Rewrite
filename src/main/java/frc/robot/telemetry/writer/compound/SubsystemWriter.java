package frc.robot.telemetry.writer.compound;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.writer.StringWriter;

public class SubsystemWriter<T extends Subsystem> implements AutoCloseable {

    private final T subsystem;

    private final StringWriter currentCommandWriter;
    private final StringWriter defaultCommandWriter;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public SubsystemWriter(T _subsystem, String table) {
        subsystem = _subsystem;

        currentCommandWriter = Telemetry.makeStringWriterEx(table, "currentCommand", null, true, false);
        defaultCommandWriter = Telemetry.makeStringWriterEx(table, "defaultCommand", null, true, false);
    }

    public void update() {
        Command currentCommand = subsystem.getCurrentCommand();
        currentCommandWriter.set(currentCommand == null ? null : currentCommand.getName());

        Command defaultCommand = subsystem.getDefaultCommand();
        defaultCommandWriter.set(defaultCommand == null ? null : defaultCommand.getName());
    }

    @Override
    public void close() {
        currentCommandWriter.close();
        defaultCommandWriter.close();
    }
}
