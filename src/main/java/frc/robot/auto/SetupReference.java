package frc.robot.auto;

import static frc.robot.constants.SetupReferenceConstants.*;

import org.opencv.core.Mat;

public enum SetupReference {
    NONE(noneImg),
    DEBUG(debugImg),
    LEFT_TRENCH(leftTrenchImg),
    RIGHT_TRENCH(rightTrenchImg);

    public final Mat image;

    private SetupReference(Mat _image) {
        image = _image;
    }
}
