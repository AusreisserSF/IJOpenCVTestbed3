package org.firstinspires.ftc.teamcode.auto.xml;

// Input parameters to OpenCV Watershed.
public class WatershedParametersFtc {

    public final WatershedDistanceParameters watershedDistanceParameters;

    public WatershedParametersFtc(WatershedDistanceParameters pWatershedDistanceParameters) {
        watershedDistanceParameters = pWatershedDistanceParameters;
    }

    public static class WatershedDistanceParameters {
        public final VisionParameters.GrayParameters redGrayParameters;
        public final VisionParameters.GrayParameters blueGrayParameters;

        public WatershedDistanceParameters(VisionParameters.GrayParameters pRedGrayParameters,
                                                VisionParameters.GrayParameters pBlueGrayParameters) {
            redGrayParameters = pRedGrayParameters;
            blueGrayParameters = pBlueGrayParameters;
        }
    }

}