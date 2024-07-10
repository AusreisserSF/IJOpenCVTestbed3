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

    // Where to look for an object within an image ROI. The LEFT_WINDOW
    // always starts at the 0 x-coordinate the image ROI, which may or
    // may not coincide with the 0 x-coordinate of the full image.
    //**TODO Put an explanation somewhere ...
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
