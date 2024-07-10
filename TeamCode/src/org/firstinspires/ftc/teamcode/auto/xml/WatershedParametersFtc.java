package org.firstinspires.ftc.teamcode.auto.xml;

// Input parameters to OpenCV Watershed.
public class WatershedParametersFtc {

    public final WatershedDistanceParameters watershedDistanceParameters;

    public WatershedParametersFtc(WatershedDistanceParameters pWatershedDistanceParameters) {
        watershedDistanceParameters = pWatershedDistanceParameters;
    }

    //**TODO TEMP leave the pixel count criteria parameters in place. We may need them later.
    public static class WatershedDistanceParameters {
        public final VisionParameters.GrayParameters redGrayParameters;
        public final int redMinWhitePixelCount;
        public final VisionParameters.GrayParameters blueGrayParameters;
        public final int blueMinWhitePixelCount;

        public WatershedDistanceParameters(VisionParameters.GrayParameters pRedGrayParameters, int pRedMinWhitePixelCount,
                                                VisionParameters.GrayParameters pBlueGrayParameters, int pBlueMinWhitePixelCount) {
            redGrayParameters = pRedGrayParameters;
            redMinWhitePixelCount = pRedMinWhitePixelCount;
            blueGrayParameters = pBlueGrayParameters;
            blueMinWhitePixelCount = pBlueMinWhitePixelCount;
        }
    }

}