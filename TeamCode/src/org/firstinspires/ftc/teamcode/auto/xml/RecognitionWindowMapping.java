package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.opencv.core.Rect;

import java.util.EnumMap;

// When the FTC game, such as Center Stage, requires us to recognize
// an object in one of three locations we can choose to crop our ROI
// to provide "recognition windows" onto just two of the locations
// and detect the object in one of the two windows or infer its
// presence outside the ROI. We set the ROI based on the OpMode. We
// call the two windows onto the ROI the LEFT recognition window and
// the RIGHT recognition window. The inferred third window is called
// WINDOW_NPOS. The LEFT window always starts at the 0 x-coordinate
// the ROI, which may or may not coincide with the 0 x-coordinate of
// the full image. The mapping of each recognition window to the
// corresponding object location - see the field recognitionWindows
// below - is captured from the children of the <recognition_window>
// element in the RobotAction.xml file for OpModes that need this
// mapping. See RecognitionWindowMapping.java.
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
