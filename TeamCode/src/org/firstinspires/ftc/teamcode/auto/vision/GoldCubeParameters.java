package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters to gold cube recognition.
public class GoldCubeParameters {

    public final VisionParameters.GrayParameters grayscaleParameters;
    public final VisionParameters.HSVParameters hsvParameters;

    public final BoundingBoxCriteria boundingBoxCriteria;

    public GoldCubeParameters(VisionParameters.GrayParameters pGrayscaleParameters,
                              VisionParameters.HSVParameters pHSVParameters,
                              BoundingBoxCriteria pCriteria) {
        grayscaleParameters = pGrayscaleParameters;
        hsvParameters = pHSVParameters;
        boundingBoxCriteria = pCriteria;
    }

    public static class BoundingBoxCriteria {
        public final double minBoundingBoxArea;
        public final double maxBoundingBoxArea;

        public BoundingBoxCriteria(double pMin, double pMax) {
            minBoundingBoxArea = pMin;
            maxBoundingBoxArea = pMax;
        }

    }

}