package org.firstinspires.ftc.teamcode.auto.xml;

// Input parameters to gold cube recognition.
public class LABTesterParameters {

    public final VisionParameters.GrayParameters grayscaleParameters;
    public final VisionParameters.LABParameters labParameters;

    // The L*a*b* parameters have already been converted from their
    // original values to their OpenCV equivalents.
    public LABTesterParameters(VisionParameters.GrayParameters pGrayscaleParameters,
                               VisionParameters.LABParameters pLABParameters) {
        grayscaleParameters = pGrayscaleParameters;
        labParameters = pLABParameters;
    }

}