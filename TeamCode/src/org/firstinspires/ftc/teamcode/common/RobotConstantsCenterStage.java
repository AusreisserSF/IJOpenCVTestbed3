package org.firstinspires.ftc.teamcode.common;

public class RobotConstantsCenterStage {

    public enum OpMode {
        // Autonomous OpModes
        BLUE_A2(OpModeType.COMPETITION),
        BLUE_A4(OpModeType.COMPETITION),
        RED_F2(OpModeType.COMPETITION),
        RED_F4(OpModeType.COMPETITION),

        TEST(OpModeType.AUTO_TEST), TEST_PRE_MATCH(OpModeType.AUTO_TEST),
        AUTO_NO_DRIVE(OpModeType.AUTO_TEST),

        // TeleOp OpModes
        TELEOP_NO_DRIVE(OpModeType.TELEOP_TEST);

        public enum OpModeType {COMPETITION, AUTO_TEST, TELEOP_TEST, PSEUDO_OPMODE}
        private final OpModeType opModeType;

        OpMode(OpModeType pOpModeType) {
            opModeType = pOpModeType;
        }

        public OpModeType getOpModeType() {
            return opModeType;
        }
    }

    // Reference implementation of the gold cube.
    public enum GoldCubeRecognitionPath {
        RED_CHANNEL_GRAYSCALE, COLOR
    }

}