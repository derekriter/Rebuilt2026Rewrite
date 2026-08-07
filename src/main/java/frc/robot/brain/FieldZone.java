package frc.robot.brain;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.FieldConstants;

public enum FieldZone {
    BLUE(Meters.zero(), FieldConstants.blueZoneEdge),
    NEUTRAL(FieldConstants.blueZoneEdge, FieldConstants.redZoneEdge),
    RED(FieldConstants.redZoneEdge, FieldConstants.fieldXBound);

    public final Distance leftBound;
    public final Distance rightBound;

    private FieldZone(Distance _leftBound, Distance _rightBound) {
        leftBound = _leftBound;
        rightBound = _rightBound;
    }

    public static FieldZone fromRobotX(Distance robotX) {
        if (robotX.lt(BLUE.rightBound)) {
            return BLUE;
        } else if (robotX.lte(NEUTRAL.rightBound)) {
            return NEUTRAL;
        } else {
            return RED;
        }
    }
}
