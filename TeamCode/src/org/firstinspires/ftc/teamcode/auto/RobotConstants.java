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

    //## See comments in RecognitionWindowMapping.java regarding
    // the enums RecognitionWindow and ObjectLocation and their
    // relationship.
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
