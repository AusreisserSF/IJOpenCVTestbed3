package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.*;
import java.io.File;
import java.io.IOException;

// Class whose job it is to read an XML file that contains all of the information
// needed to perform simple sample recognition for the FTC game IntoTheDeep.
public class SampleContoursParametersXML {
    public static final String TAG = SampleContoursParametersXML.class.getSimpleName();
    private static final String SAMPLE_FILE_NAME = "SampleContoursParameters.xml";

    private final SampleContoursParameters sampleContoursParameters;

    public SampleContoursParametersXML(String pXMLDir) {
        Node sample_parameters_node;
        try {
            String xmlFilePath = pXMLDir + SAMPLE_FILE_NAME;
            RobotLogCommon.c(TAG, "Parsing " + SAMPLE_FILE_NAME);

            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document document = dBuilder.parse(new File(xmlFilePath));
            XPathFactory xpathFactory = XPathFactory.newInstance();
            XPath xpath = xpathFactory.newXPath();

            // Point to the first node.
            XPathExpression expr = xpath.compile("//sample_contours_parameters");
            sample_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
            if (sample_parameters_node == null)
                throw new AutonomousRobotException(TAG, "Element '//sample_contours_parameters' not found");

        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        } catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
        } catch (XPathExpressionException xex) {
            throw new AutonomousRobotException(TAG, "XPath Exception " + xex.getMessage());
        }

        // Point to <rgb_channel_grayscale>.
        Node rgb_node = sample_parameters_node.getFirstChild();
        rgb_node = XMLUtils.getNextElement(rgb_node);
        if ((rgb_node == null) || !rgb_node.getNodeName().equals("rgb_channel_grayscale"))
            throw new AutonomousRobotException(TAG, "Element 'rgb_channel_grayscale' not found");

        // Process the children of <rgb_channel_grayscale>.
        // Point to <RED> for the red rgb channel parameters.
        Node rgb_red_node = rgb_node.getFirstChild();
        rgb_red_node = XMLUtils.getNextElement(rgb_red_node);
        if ((rgb_red_node == null) || !rgb_red_node.getNodeName().equals("RED"))
            throw new AutonomousRobotException(TAG, "Element 'RED' not found");

        // Point to <gray_parameters>
        Node red_gray_node = rgb_red_node.getFirstChild();
        red_gray_node = XMLUtils.getNextElement(red_gray_node);
        if ((red_gray_node == null) || !red_gray_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'RED/gray_parameters' not found");

        VisionParameters.GrayParameters redGrayParameters = ImageXML.parseGrayParameters(red_gray_node);

        // Point to <GREEN> for the green rgb channel parameters.
        Node rgb_green_node = rgb_red_node.getNextSibling();
        rgb_green_node = XMLUtils.getNextElement(rgb_green_node);
        if ((rgb_green_node == null) || !rgb_green_node.getNodeName().equals("GREEN"))
            throw new AutonomousRobotException(TAG, "Element 'GREEN' not found");

        // Point to <gray_parameters>
        Node green_gray_node = rgb_green_node.getFirstChild();
        green_gray_node = XMLUtils.getNextElement(green_gray_node);
        if ((green_gray_node == null) || !green_gray_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'GREEN/gray_parameters' not found");

        VisionParameters.GrayParameters greenGrayParameters = ImageXML.parseGrayParameters(green_gray_node);

        SampleContoursParameters.RGBChannelGrayscaleParameters sampleGrayscaleParameters =
                new SampleContoursParameters.RGBChannelGrayscaleParameters(redGrayParameters, greenGrayParameters);

        // Point to <hsv_color>.
        Node hsv_node = rgb_node.getNextSibling();
        hsv_node = XMLUtils.getNextElement(hsv_node);
        if ((hsv_node == null) || !hsv_node.getNodeName().equals("hsv_color"))
            throw new AutonomousRobotException(TAG, "Element 'hsv_color' not found");

        // Process the children of <hsv_color>.
        //## At this point we only support blue.
        // Point to <RED> for the blue lab parameters.
        Node hsv_blue_node = hsv_node.getFirstChild();
        hsv_blue_node = XMLUtils.getNextElement(hsv_blue_node);
        if ((hsv_blue_node == null) || !hsv_blue_node.getNodeName().equals("BLUE"))
            throw new AutonomousRobotException(TAG, "Element 'hsv_color/BLUE' not found");

        // Point to <hsv_parameters>
        Node blue_hsv_parameters_node = hsv_blue_node.getFirstChild();
        blue_hsv_parameters_node = XMLUtils.getNextElement(blue_hsv_parameters_node);
        if ((blue_hsv_parameters_node == null) || !blue_hsv_parameters_node.getNodeName().equals("hsv_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'RED/lab_parameters' not found");

        VisionParameters.HSVParameters blueHSVParameters = ImageXML.parseHSVParameters(blue_hsv_parameters_node);
        SampleContoursParameters.HSVColorParameters hsvColorParameters =
                new SampleContoursParameters.HSVColorParameters(blueHSVParameters);

        sampleContoursParameters = new SampleContoursParameters(sampleGrayscaleParameters, hsvColorParameters);
    }

    public SampleContoursParameters getSampleContoursParameters() {
        return sampleContoursParameters;
    }

}

