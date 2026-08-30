package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Celsius;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.IntakeConstants.LeftDeployerConstants;
import frc.robot.constants.IntakeConstants.RightDeployerConstants;
import frc.robot.constants.IntakeConstants.RollerConstants;
import frc.robot.constants.Overrides;
import frc.robot.subsystems.intake.IIntakeIO.IntakeIOInputs;
import frc.robot.util.AlertUtils;
import frc.robot.util.BlankValues;
import frc.robot.util.Console;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final IIntakeIO io_nl;
    private IntakeIOInputs inputs = new IntakeIOInputs();

    private final Alert rollerCANAlert = AlertUtils.makeCANFailureAlert("roller"),
            rollerBreakerAlert = AlertUtils.makeBreakerTripAlert("roller"),
            rollerTempWarnAlert = AlertUtils.makeTempWarnAlert("roller"),
            rollerThermalShutdownAlert = AlertUtils.makeThermalShutdownAlert("roller"),
            leftDeployerBreakerAlert = AlertUtils.makeBreakerTripAlert("leftDeployer"),
            rightDeployerBreakerAlert = AlertUtils.makeBreakerTripAlert("rightDeployer");

    private boolean rollerConnectedLast = false;
    private boolean rollerBreaker = false;
    private boolean rollerBreakerLast = false;
    private boolean rollerThermalShutdown = false;
    private boolean rollerThermalShutdownLast = false;

    private boolean leftDeployerBreaker = false;
    private boolean leftDeployerBreakerLast = false;

    private boolean rightDeployerBreaker = false;
    private boolean rightDeployerBreakerLast = false;

    private boolean hasDeployed = false;

    public Intake(IIntakeIO _io_nl) {
        io_nl = _io_nl;
        if (io_nl == null) {
            Logger.recordOutput("CAN/roller_" + RollerConstants.canID, false);
            AlertUtils.makeSystemDisabledAlert("intake").set(true);
        } else {
            if (Overrides.disableIntakeSafety) {
                AlertUtils.makeSafetyDisabledAlert("intake").set(true);
            }

            Logger.recordOutput("Intake/hasDeployed", false);
        }
    }

    @Override
    public void periodic() {
        if (io_nl != null) {
            io_nl.updateInputs(inputs);
            Logger.processInputs("IntakeInputs", inputs);

            // roller
            {
                rollerBreaker = RobotContainer.instance().pdh.isBreakerTripped(RollerConstants.channelID);

                Logger.recordOutput("CAN/roller_" + RollerConstants.canID, inputs.rollerConnected);
                rollerCANAlert.set(!inputs.rollerConnected && !rollerBreaker);
                if (inputs.rollerConnected != rollerConnectedLast) {
                    if (inputs.rollerConnected) {
                        Console.reportCANConnect("roller", RollerConstants.canID, RollerConstants.channelID);
                    } else {
                        Console.reportCANDisconnect("roller", RollerConstants.canID, RollerConstants.channelID);
                    }
                }
                Logger.recordOutput("Intake/Roller/breakerTripped", rollerBreaker);
                rollerBreakerAlert.set(rollerBreaker);
                if (rollerBreaker != rollerBreakerLast) {
                    if (rollerBreaker) {
                        Console.reportBreakerTrip("roller", RollerConstants.canID, RollerConstants.channelID);
                    } else {
                        Console.reportBreakerReset("roller", RollerConstants.canID, RollerConstants.channelID);
                    }
                }

                if (!Overrides.disableIntakeSafety && inputs.rollerConnected) {
                    boolean gettingToasty = inputs.rollerTemp_C >= RollerConstants.tempWarnThreshold.in(Celsius);
                    boolean overheating = inputs.rollerTemp_C >= RollerConstants.thermalShutdownThreshold.in(Celsius);
                    rollerThermalShutdown = (overheating || rollerThermalShutdown) && gettingToasty;

                    rollerTempWarnAlert.set(gettingToasty && !overheating);
                    Logger.recordOutput("Intake/Roller/thermalShutdown", rollerThermalShutdown);
                    rollerThermalShutdownAlert.set(rollerThermalShutdown);
                }

                if (rollerThermalShutdown != rollerThermalShutdownLast) {
                    if (rollerThermalShutdown) {
                        Console.reportThermalShutdownTrigger(
                                "roller", RollerConstants.canID, RollerConstants.channelID);

                        stopRoller();
                    } else {
                        Console.reportThermalShutdownRelease(
                                "roller", RollerConstants.canID, RollerConstants.channelID);
                    }
                }

                rollerConnectedLast = inputs.rollerConnected;
                rollerBreakerLast = rollerBreaker;
                rollerThermalShutdownLast = rollerThermalShutdown;
            }

            // left deployer
            {
                leftDeployerBreaker = RobotContainer.instance().pdh.isBreakerTripped(LeftDeployerConstants.channelID);

                Logger.recordOutput("Intake/LeftDeployer/breakerTripped", leftDeployerBreaker);
                leftDeployerBreakerAlert.set(leftDeployerBreaker);
                if (leftDeployerBreaker != leftDeployerBreakerLast) {
                    if (leftDeployerBreaker) {
                        Console.reportBreakerTripNoCAN("leftDeployer", LeftDeployerConstants.channelID);
                    } else {
                        Console.reportBreakerResetNoCAN("leftDeployer", LeftDeployerConstants.channelID);
                    }
                }

                leftDeployerBreakerLast = leftDeployerBreaker;
            }

            // right deployer
            {
                rightDeployerBreaker = RobotContainer.instance().pdh.isBreakerTripped(RightDeployerConstants.channelID);

                Logger.recordOutput("Intake/LeftDeployer/breakerTripped", rightDeployerBreaker);
                rightDeployerBreakerAlert.set(rightDeployerBreaker);
                if (rightDeployerBreaker != rightDeployerBreakerLast) {
                    if (rightDeployerBreaker) {
                        Console.reportBreakerTripNoCAN("rightDeployer", RightDeployerConstants.channelID);
                    } else {
                        Console.reportBreakerResetNoCAN("rightDeployer", RightDeployerConstants.channelID);
                    }
                }

                rightDeployerBreakerLast = rightDeployerBreaker;
            }
        }

        Command currentCommand = getCurrentCommand();
        Logger.recordOutput(
                "Intake/currentCommand", currentCommand == null ? BlankValues.string : currentCommand.getName());

        Command defaultCommand = getDefaultCommand();
        Logger.recordOutput(
                "Intake/defaultCommand", defaultCommand == null ? BlankValues.string : defaultCommand.getName());
    }

    public void report(IntakeReport report) {
        if (Overrides.disableIntakeSafety) {
            report.rollerOperational = true;
            report.deployerOperational = true;
        } else if (io_nl == null) {
            report.rollerOperational = false;
            report.deployerOperational = false;
        } else {
            report.rollerOperational = inputs.rollerConnected && !rollerBreaker && !rollerThermalShutdown;
            report.deployerOperational = !leftDeployerBreaker && !rightDeployerBreaker;
        }
        report.hasDeployed = hasDeployed;
    }

    public void deploy() {
        if (io_nl == null || hasDeployed) return;

        io_nl.setLeftDeployerPosition(LeftDeployerConstants.deployedPosition);
        io_nl.setRightDeployerPosition(RightDeployerConstants.deployedPosition);

        hasDeployed = true;
        Logger.recordOutput("Intake/hasDeployed", true);
    }

    public void setRollerVoltage(double voltage_V) {
        if (io_nl == null) return;
        if (rollerThermalShutdown && Math.abs(voltage_V) > 1e-6) return;

        io_nl.setRollerVoltage(voltage_V);
    }

    public void stopRoller() {
        setRollerVoltage(0);
    }

    public boolean isDeployed() {
        return hasDeployed;
    }
}
