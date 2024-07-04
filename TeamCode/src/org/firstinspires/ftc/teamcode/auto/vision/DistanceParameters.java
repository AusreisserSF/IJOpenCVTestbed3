package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters to DistanceTransform recognition.
public class DistanceParameters {

    public final ColorChannelBrightSpotParameters colorChannelBrightSpotParameters;
    public final ColorChannelContoursParameters colorChannelContoursParameters;
    public final ColorChannelPixelCountParameters colorChannelPixelCountParameters;

    public DistanceParameters(ColorChannelBrightSpotParameters pColorChannelBrightSpotParameters,
                              ColorChannelContoursParameters pColorChannelContoursParameters,
                              ColorChannelPixelCountParameters pColorChannelPixelCountParameters) {

        colorChannelBrightSpotParameters = pColorChannelBrightSpotParameters;
        colorChannelContoursParameters = pColorChannelContoursParameters;
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

    public static class ColorChannelContoursParameters {
        public final VisionParameters.GrayParameters grayParameters;
        public final double minArea;
        public final double maxArea;

        public ColorChannelContoursParameters(VisionParameters.GrayParameters pGrayParameters,
                                              double pMinArea, double pMaxArea) {
            grayParameters = pGrayParameters;
            minArea = pMinArea;
            maxArea = pMaxArea;
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