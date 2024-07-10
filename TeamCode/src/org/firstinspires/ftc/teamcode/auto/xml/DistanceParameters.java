package org.firstinspires.ftc.teamcode.auto.xml;

// Input parameters to DistanceTransform recognition.
public class DistanceParameters {
    public final ColorChannelBrightSpotParameters colorChannelBrightSpotParameters;
    public final ColorChannelPixelCountParameters colorChannelPixelCountParameters;

    public DistanceParameters(ColorChannelBrightSpotParameters pColorChannelBrightSpotParameters,
                              ColorChannelPixelCountParameters pColorChannelPixelCountParameters) {

        colorChannelBrightSpotParameters = pColorChannelBrightSpotParameters;
        colorChannelPixelCountParameters = pColorChannelPixelCountParameters;
    }

    public static class ColorChannelBrightSpotParameters {
        public final VisionParameters.GrayParameters redGrayParameters;
        public final VisionParameters.GrayParameters blueGrayParameters;

        public ColorChannelBrightSpotParameters(VisionParameters.GrayParameters pRedGrayParameters,
                                                VisionParameters.GrayParameters pBlueGrayParameters) {
            redGrayParameters = pRedGrayParameters;
            blueGrayParameters = pBlueGrayParameters;
        }
    }

    public static class ColorChannelPixelCountParameters {
        public final VisionParameters.GrayParameters redGrayParameters;
        public final int redMinWhitePixelCount;
        public final VisionParameters.GrayParameters blueGrayParameters;
        public final int blueMinWhitePixelCount;

        public ColorChannelPixelCountParameters(VisionParameters.GrayParameters pRedGrayParameters, int pRedMinWhitePixelCount,
                                                VisionParameters.GrayParameters pBlueGrayParameters, int pBlueMinWhitePixelCount) {
            redGrayParameters = pRedGrayParameters;
            redMinWhitePixelCount = pRedMinWhitePixelCount;
            blueGrayParameters = pBlueGrayParameters;
            blueMinWhitePixelCount = pBlueMinWhitePixelCount;
        }
    }

}