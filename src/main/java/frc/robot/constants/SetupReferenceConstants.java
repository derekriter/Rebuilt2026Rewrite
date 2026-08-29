package frc.robot.constants;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.Console;
import java.nio.file.Paths;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

public final class SetupReferenceConstants {

    public static final Mat noneImg = safeLoadImage("setupImages/none.jpg");
    public static final Mat leftTrenchImg = safeLoadImage("setupImages/leftTrench.jpg");
    public static final Mat rightTrenchImg = safeLoadImage("setupImages/rightTrench.jpg");

    private static Mat safeLoadImage(String deployRelativePath) {
        try {
            return Imgcodecs.imread(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), deployRelativePath)
                    .toString());
        } catch (Throwable e) {
            Console.reportError(e, true);

            return new Mat();
        }
    }

    private SetupReferenceConstants() {}
}
