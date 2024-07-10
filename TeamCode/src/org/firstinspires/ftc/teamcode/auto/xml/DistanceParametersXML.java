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
// needed to perform an OpenCV Watershed operation.
public class DistanceParametersXML {
    public static final String TAG = DistanceParametersXML.class.getSimpleName();
    private static final String DISTANCE_FILE_NAME = "DistanceParameters.xml";

    private final DistanceParameters distanceParameters;

    public DistanceParametersXML(String pXMLDir) {
        Node distance_parameters_node;
        try {
            String xmlFilePath = pXMLDir + DISTANCE_FILE_NAME;
            RobotLogCommon.c(TAG, "Parsing " + DISTANCE_FILE_NAME);

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
            XPathExpression expr = xpath.compile("//distance_parameters");
            distance_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
            if (distance_parameters_node == null)
                throw new AutonomousRobotException(TAG, "Element '//distance_parameters' not found");

        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        } catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
        } catch (XPathExpressionException xex) {
            throw new AutonomousRobotException(TAG, "XPath Exception " + xex.getMessage());
        }

        // Point to <color_channel_bright_spot>
        Node bright_spot_node = distance_parameters_node.getFirstChild();
        bright_spot_node = XMLUtils.getNextElement(bright_spot_node);
        if ((bright_spot_node == null) || !bright_spot_node.getNodeName().equals("color_channel_bright_spot"))
            throw new AutonomousRobotException(TAG, "Element 'color_channel_bright_spot' not found");

        // Point to <RED> for the red alliance parameters.
        Node red_bright_node = bright_spot_node.getFirstChild();
        red_bright_node = XMLUtils.getNextElement(red_bright_node);
        if ((red_bright_node == null) || !red_bright_node.getNodeName().equals("RED"))
            throw new AutonomousRobotException(TAG, "Element 'RED' under 'color_channel_bright_spot' not found");

        // Point to <gray_parameters>
        Node red_bright_gray_node = red_bright_node.getFirstChild();
        red_bright_gray_node = XMLUtils.getNextElement(red_bright_gray_node);
        if ((red_bright_gray_node == null) || !red_bright_gray_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'color_channel_bright_spot/RED/gray_parameters' not found");

        VisionParameters.GrayParameters redBrightSpotGrayParameters = ImageXML.parseGrayParameters(red_bright_gray_node);

        // Point to <BLUE> for the blue alliance parameters.
        Node blue_bright_node = red_bright_node.getNextSibling();
        blue_bright_node = XMLUtils.getNextElement(blue_bright_node);
        if ((blue_bright_node == null) || !blue_bright_node.getNodeName().equals("BLUE"))
            throw new AutonomousRobotException(TAG, "Element 'BLUE' under 'color_channel_bright_spot' not found");

        // Point to <gray_parameters>
        Node blue_bright_gray_node = blue_bright_node.getFirstChild();
        blue_bright_gray_node = XMLUtils.getNextElement(blue_bright_gray_node);
        if ((blue_bright_gray_node == null) || !blue_bright_gray_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'color_channel_bright_spot/BLUE/gray_parameters' not found");

        VisionParameters.GrayParameters blueBrightSpotGrayParameters = ImageXML.parseGrayParameters(blue_bright_gray_node);

        DistanceParameters.ColorChannelBrightSpotParameters brightSpotParameters =
                new DistanceParameters.ColorChannelBrightSpotParameters(redBrightSpotGrayParameters,
                        blueBrightSpotGrayParameters);

        Node pixel_count_node = bright_spot_node.getNextSibling();
        pixel_count_node = XMLUtils.getNextElement(pixel_count_node);
        if ((pixel_count_node == null) || !pixel_count_node.getNodeName().equals("color_channel_pixel_count"))
            throw new AutonomousRobotException(TAG, "Element 'color_channel_pixel_count' not found");

        // Point to <RED> for the red alliance parameters.
        Node red_pixel_count_node = pixel_count_node.getFirstChild();
        red_pixel_count_node = XMLUtils.getNextElement(red_pixel_count_node);
        if ((red_pixel_count_node == null) || !red_pixel_count_node.getNodeName().equals("RED"))
            throw new AutonomousRobotException(TAG, "Element 'RED' under 'color_channel_pixel_count' not found");

        // Point to <gray_parameters>
        Node red_pixel_count_gray_node = red_pixel_count_node.getFirstChild();
        red_pixel_count_gray_node = XMLUtils.getNextElement(red_pixel_count_gray_node);
        if ((red_pixel_count_gray_node == null) || !red_pixel_count_gray_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'color_channel_pixel_count/RED/gray_parameters' not found");

        VisionParameters.GrayParameters redPixelCountGrayParameters = ImageXML.parseGrayParameters(red_pixel_count_gray_node);

        // Point to the criteria for the red pixel count.
        Node red_pixel_count_criteria_node = red_pixel_count_gray_node.getNextSibling();
        red_pixel_count_criteria_node = XMLUtils.getNextElement(red_pixel_count_criteria_node);
        if (red_pixel_count_criteria_node == null)
            throw new AutonomousRobotException(TAG, "Element 'color_channel_pixel_count/RED/criteria' not found");

        // Parse the <min_white_pixel_count> element.
        Node red_min_pixels_node = red_pixel_count_criteria_node.getFirstChild();
        red_min_pixels_node = XMLUtils.getNextElement(red_min_pixels_node);
        if (red_min_pixels_node == null || !red_min_pixels_node.getNodeName().equals("min_white_pixel_count") ||
                red_min_pixels_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'color_channel_pixel_count/RED/criteria/min_white_pixel_count' not found or empty");

        String redMinPixelsText = red_min_pixels_node.getTextContent();
        int redMinPixelCount;
        try {
            redMinPixelCount = Integer.parseInt(redMinPixelsText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'color_channel_pixel_count/RED/criteria/min_white_pixel_count'");
        }

        // Point to <BLUE> for the blue alliance parameters.
        Node blue_pixel_count_node = red_pixel_count_node.getNextSibling();
        blue_pixel_count_node = XMLUtils.getNextElement(blue_pixel_count_node);
        if ((blue_pixel_count_node == null) || !blue_pixel_count_node.getNodeName().equals("BLUE"))
            throw new AutonomousRobotException(TAG, "Element 'BLUE' under 'color_channel_pixel_count' not found");

        // Point to <gray_parameters>
        Node blue_pixel_count_gray_node = blue_pixel_count_node.getFirstChild();
        blue_pixel_count_gray_node = XMLUtils.getNextElement(blue_pixel_count_gray_node);
        if ((blue_pixel_count_gray_node == null) || !blue_pixel_count_gray_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'color_channel_pixel_count/BLUE/gray_parameters' not found");

        VisionParameters.GrayParameters bluePixelCountGrayParameters = ImageXML.parseGrayParameters(blue_pixel_count_gray_node);

        // Point to the criteria for the blue pixel count.
        Node blue_pixel_count_criteria_node = blue_pixel_count_gray_node.getNextSibling();
        blue_pixel_count_criteria_node = XMLUtils.getNextElement(blue_pixel_count_criteria_node);
        if (blue_pixel_count_criteria_node == null)
            throw new AutonomousRobotException(TAG, "Element 'color_channel_pixel_count/BLUE/criteria' not found");

        // Parse the <min_white_pixel_count> element.
        Node blue_min_pixels_node = blue_pixel_count_criteria_node.getFirstChild();
        blue_min_pixels_node = XMLUtils.getNextElement(blue_min_pixels_node);
        if (blue_min_pixels_node == null || !blue_min_pixels_node.getNodeName().equals("min_white_pixel_count") ||
                blue_min_pixels_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'color_channel_pixel_count/BLUE/criteria/min_white_pixel_count' not found or empty");

        String blueMinPixelsText = blue_min_pixels_node.getTextContent();
        int blueMinPixelCount;
        try {
            blueMinPixelCount = Integer.parseInt(blueMinPixelsText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'color_channel_pixel_count/BLUE/criteria/min_white_pixel_count'");
        }

        DistanceParameters.ColorChannelPixelCountParameters colorChannelPixelCountParameters =
                new DistanceParameters.ColorChannelPixelCountParameters(redPixelCountGrayParameters, redMinPixelCount,
                        bluePixelCountGrayParameters, blueMinPixelCount);

        distanceParameters = new DistanceParameters(brightSpotParameters, colorChannelPixelCountParameters);
    }

    public DistanceParameters getDistanceParameters() {
        return distanceParameters;
    }

}

