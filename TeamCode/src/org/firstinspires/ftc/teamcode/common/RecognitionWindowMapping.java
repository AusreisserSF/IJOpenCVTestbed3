package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.opencv.core.Rect;

import java.util.EnumMap;

public class RecognitionWindowMapping {
    //**TODO Are you sure you need this here? If not, then the only thing left is the EnumMap.
    public final VisionParameters.ImageParameters imageParameters;
    public final EnumMap<RobotConstants.RecognitionWindow, Pair<Rect, RobotConstants.ObjectLocation>> recognitionWindows;

    public RecognitionWindowMapping(VisionParameters.ImageParameters pImageParameters,
                                    EnumMap<RobotConstants.RecognitionWindow, Pair<Rect, RobotConstants.ObjectLocation>> pRecognitionWindows) {
        imageParameters = pImageParameters;
        recognitionWindows = pRecognitionWindows;
    }

}
