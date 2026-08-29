package frc.robot.constants;

import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.Console;
import java.nio.file.Paths;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

public final class SetupReferenceConstants {

    public static final Mat noneImg = safeLoadImage("setupImages/none.jpg");
    public static final Mat debugImg = safeLoadImage("setupImages/debug.jpg");
    public static final Mat leftTrenchImg = safeLoadImage("setupImages/placeholder.jpg");
    public static final Mat rightTrenchImg = safeLoadImage("setupImages/placeholder.jpg");
    public static final Mat voltImg = safeLoadImage("setupImages/volt.jpg");

    private static Mat safeLoadImage(String deployRelativePath) {
        try {
            OpenCvLoader.forceLoad();

            Mat img;
            if ((img = Imgcodecs.imread(
                            Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), deployRelativePath)
                                    .toString()))
                    .empty()) {
                img = Mat.zeros(1, 1, CvType.CV_8UC3);
            }

            return img;
        } catch (Throwable e) {
            Console.reportError(e, true);
            return Mat.zeros(1, 1, CvType.CV_8UC3);
        }
    }

    private SetupReferenceConstants() {}
}
