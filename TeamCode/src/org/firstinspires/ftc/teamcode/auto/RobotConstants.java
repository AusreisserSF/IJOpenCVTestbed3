package org.firstinspires.ftc.teamcode.auto;

public class RobotConstants {
    public static final String ACTION_FILENAME = "RobotAction.xml";

    public enum Alliance {
        RED, BLUE, NONE
    }

    public enum OpMode {
        TEST(OpModeType.AUTO_TEST);

        public enum OpModeType {COMPETITION, AUTO_TEST}

        private final OpModeType opModeType;

        OpMode(OpModeType pOpModeType) {
            opModeType = pOpModeType;
        }

        public OpModeType getOpModeType() {
            return opModeType;
        }
    }

    //**TODO Move this to RecognitionWindowMapping and reference
    // from here ...
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
    // corresponding object location - see the ObjectLocation enum below -
    // is captured in the children of the <recognition_window> element
    // in the RobotAction.xml file for OpModes that need this mapping.
    public enum RecognitionWindow {
        LEFT, RIGHT, WINDOW_NPOS
    }

    // The relative location of the object within the full image.
    public enum ObjectLocation {
        LEFT, CENTER, RIGHT, LOCATION_NPOS
    }

    public enum RecognitionResults {
        RECOGNITION_INTERNAL_ERROR, RECOGNITION_SUCCESSFUL, RECOGNITION_UNSUCCESSFUL
    }

}
