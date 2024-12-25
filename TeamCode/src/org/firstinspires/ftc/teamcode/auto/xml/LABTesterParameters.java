package org.firstinspires.ftc.teamcode.auto.xml;

// Input parameters to gold cube recognition.
public class LABTesterParameters {

    public final VisionParameters.GrayParameters grayscaleParameters;
    public final VisionParameters.LABParameters labParameters;

    public LABTesterParameters(VisionParameters.GrayParameters pGrayscaleParameters,
                               VisionParameters.LABParameters pLABParameters) {
        grayscaleParameters = pGrayscaleParameters;
        labParameters = pLABParameters;
    }

}