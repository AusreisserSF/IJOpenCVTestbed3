package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.opencv.core.Rect;

import java.util.EnumMap;

public class RecognitionWindowMapping {
    //## ImageParameters are not used in IntelliJ but are used in Android Studio,
    // e.g. FtcCenterStage SpikeWindowRendering, which shows the spike windows on
    // the Driver Station but which does not have direct access to RobotAction.xml.
    public final VisionParameters.ImageParameters imageParameters;
    public final EnumMap<RobotConstants.RecognitionWindow, Pair<Rect, RobotConstants.ObjectLocation>> recognitionWindows;

    public RecognitionWindowMapping(VisionParameters.ImageParameters pImageParameters,
                                    EnumMap<RobotConstants.RecognitionWindow, Pair<Rect, RobotConstants.ObjectLocation>> pRecognitionWindows) {
        imageParameters = pImageParameters;
        recognitionWindows = pRecognitionWindows;
    }

}
