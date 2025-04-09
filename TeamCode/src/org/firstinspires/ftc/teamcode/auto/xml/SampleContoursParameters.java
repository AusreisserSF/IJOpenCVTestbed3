package org.firstinspires.ftc.teamcode.auto.xml;

// Input parameters for IntoTheDeep sample recognition.
public class SampleContoursParameters {
    public final RGBChannelGrayscaleParameters rgbChannelGrayscaleParameters;
    public final HSVColorParameters hsvColorParameters;

    public SampleContoursParameters(RGBChannelGrayscaleParameters pSampleGrayscaleParameters,
                            HSVColorParameters pHSVColorParameters) {
        rgbChannelGrayscaleParameters = pSampleGrayscaleParameters;
        hsvColorParameters = pHSVColorParameters;
    }

    public static class RGBChannelGrayscaleParameters {
        public final VisionParameters.GrayParameters redGrayParameters;
        public final VisionParameters.GrayParameters greenGrayParameters;

        public RGBChannelGrayscaleParameters(VisionParameters.GrayParameters pRedGrayParameters,
                                             VisionParameters.GrayParameters pGreenGrayParameters) {
            redGrayParameters = pRedGrayParameters;
            greenGrayParameters = pGreenGrayParameters;
        }
    }

    public static class HSVColorParameters {
        public final VisionParameters.HSVParameters blueHSVParameters;

        public HSVColorParameters(
                VisionParameters.HSVParameters pBlueHSVParameters) {
            blueHSVParameters = pBlueHSVParameters;
        }
    }

}