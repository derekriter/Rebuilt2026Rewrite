package frc.robot.pdh;

import static frc.robot.config.PDHConfig.*;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.config.Overrides;
import frc.robot.util.AlertUtils;
import frc.robot.util.Console;
import java.util.Arrays;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.Logger;

public class PDH {

    private boolean connected = false;
    private boolean connectedLast = false;
    private boolean[] breakersTripped = new boolean[0];

    private final Alert canAlert = AlertUtils.makeCANFailureAlert("PDH");
    private final String CANKey = "CAN/PDH_" + canID;

    public PDH() {
        if (Overrides.disablePDHMonitoring) {
            Logger.recordOutput(CANKey, false);
            AlertUtils.makeSystemDisabledAlert("PDH monitoring").set(true);
        } else {
            LoggedPowerDistribution.getInstance(canID, type);

            breakersTripped = new boolean[ConduitApi.getInstance().getPDPChannelCount()];
            Arrays.fill(breakersTripped, false);
        }
    }

    public void periodic() {
        if (Overrides.disablePDHMonitoring) return;

        connected = ConduitApi.getInstance().getPDPVoltage() != 0;

        if (connected) {
            PowerDistributionFaults faults =
                    new PowerDistributionFaults((int) ConduitApi.getInstance().getPDPFaults());
            for (int i = 0; i < breakersTripped.length; i++) {
                breakersTripped[i] = faults.getBreakerFault(i);
            }
        }

        canAlert.set(!connected);
        if (connected != connectedLast) {
            if (connected) {
                Console.reportCANConnectNoChannel("PDH", canID);
            } else {
                Console.reportCANDisconnectNoChannel("PDH", canID);
            }
        }

        connectedLast = connected;
    }

    public boolean isBreakerTripped(int channel) {
        if (Overrides.disablePDHMonitoring || !connected || channel < 0 || channel >= breakersTripped.length)
            return false;

        return breakersTripped[channel];
    }
}
