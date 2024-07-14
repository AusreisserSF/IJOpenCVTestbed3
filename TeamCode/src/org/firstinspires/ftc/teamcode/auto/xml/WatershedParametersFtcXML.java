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
public class WatershedParametersFtcXML {
    public static final String TAG = WatershedParametersFtcXML.class.getSimpleName();
    private static final String WATERSHED_FTC_FILE_NAME = "WatershedParametersFtc.xml";

    private final WatershedParametersFtc watershedParametersFtc;

    public WatershedParametersFtcXML(String pXMLDir) {
        Node watershed_parameters_node;
        try {
            RobotLogCommon.c(TAG, "Parsing " + WATERSHED_FTC_FILE_NAME);

            String xmlFilePath = pXMLDir + WATERSHED_FTC_FILE_NAME;

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
            XPathExpression expr = xpath.compile("//watershed_parameters_ftc");
            watershed_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
            if (watershed_parameters_node == null)
                throw new AutonomousRobotException(TAG, "Element '//watershed_parameters_ftc' not found");

        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        } catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
        } catch (XPathExpressionException xex) {
            throw new AutonomousRobotException(TAG, "XPath Exception " + xex.getMessage());
        }

        // Point to <watershed_distance>
        Node distance_node = watershed_parameters_node.getFirstChild();
        distance_node = XMLUtils.getNextElement(distance_node);
        if ((distance_node == null) || !distance_node.getNodeName().equals("watershed_distance"))
            throw new AutonomousRobotException(TAG, "Element 'watershed_distance' not found");

        // Point to <RED> for the red alliance parameters.
        Node red_distance_node = distance_node.getFirstChild();
        red_distance_node = XMLUtils.getNextElement(red_distance_node);
        if ((red_distance_node == null) || !red_distance_node.getNodeName().equals("RED"))
            throw new AutonomousRobotException(TAG, "Element 'RED' under 'watershed_distance' not found");

        // Point to <gray_parameters>
        Node red_distance_gray_node = red_distance_node.getFirstChild();
        red_distance_gray_node = XMLUtils.getNextElement(red_distance_gray_node);
        if ((red_distance_gray_node == null) || !red_distance_gray_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'watershed_distance/RED/gray_parameters' not found");

        VisionParameters.GrayParameters redDistanceGrayParameters = ImageXML.parseGrayParameters(red_distance_gray_node);

        // Point to <BLUE> for the blue alliance parameters.
        Node blue_distance_node = red_distance_node.getNextSibling();
        blue_distance_node = XMLUtils.getNextElement(blue_distance_node);
        if ((blue_distance_node == null) || !blue_distance_node.getNodeName().equals("BLUE"))
            throw new AutonomousRobotException(TAG, "Element 'BLUE' under 'watershed_distance' not found");

        // Point to <gray_parameters>
        Node blue_distance_gray_node = blue_distance_node.getFirstChild();
        blue_distance_gray_node = XMLUtils.getNextElement(blue_distance_gray_node);
        if ((blue_distance_gray_node == null) || !blue_distance_gray_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'watershed_distance/BLUE/gray_parameters' not found");

        VisionParameters.GrayParameters bluePixelCountGrayParameters = ImageXML.parseGrayParameters(blue_distance_gray_node);

        WatershedParametersFtc.WatershedDistanceParameters watershedDistanceParameters =
                new WatershedParametersFtc.WatershedDistanceParameters(redDistanceGrayParameters,
                        bluePixelCountGrayParameters);

        watershedParametersFtc = new WatershedParametersFtc(watershedDistanceParameters);
    }

    public WatershedParametersFtc getWatershedParameters() {
        return watershedParametersFtc;
    }

}

