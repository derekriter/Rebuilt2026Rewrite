package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {
    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    public static final Translation2d redHubLoc = new Translation2d(Inches.of(651.22 - 182.11), Inches.of(158.84));
    public static final Translation2d blueHubLoc = new Translation2d(Inches.of(182.11), Inches.of(158.84));

    public static final Translation2d blueZoneTopTargetLoc =
            new Translation2d(Inches.of(156.61 / 2), Inches.of(317.69 * 3 / 4));
    public static final Translation2d blueZoneBottomTargetLoc =
            new Translation2d(Inches.of(156.61 / 2), Inches.of(317.69 / 4));
    public static final Translation2d redZoneTopTargetLoc =
            new Translation2d(Inches.of(325.61 + 167 + 156.61 / 2), Inches.of(317.69 * 3 / 4));
    public static final Translation2d redZoneBottomTargetLoc =
            new Translation2d(Inches.of(325.61 + 167 + 156.61 / 2), Inches.of(317.69 / 4));

    public static final Distance fieldXBound = Inches.of(651.22);
    public static final Distance blueZoneEdge = Inches.of(182.11);
    public static final Distance redZoneEdge = Inches.of(182.11 + 143.5 * 2);

    public static final Distance fieldYCenter = Inches.of(158.84);
    public static final Distance fieldYBound = Inches.of(317.69);

    private FieldConstants() {}
}
