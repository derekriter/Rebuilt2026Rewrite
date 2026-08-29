package frc.robot.auto;

import static frc.robot.constants.SetupReferenceConstants.*;

import org.opencv.core.Mat;

public enum SetupReference {
    NONE("None", noneImg),
    LEFT_TRENCH("Left Trench", leftTrenchImg),
    RIGHT_TRENCH("Right Trench", rightTrenchImg);

    public final String name;
    public final Mat image;

    private SetupReference(String _name, Mat _image) {
        name = _name;
        image = _image;
    }
}
