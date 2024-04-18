package frc.robot.subsystems;

import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LedConstants;; 


public class Led extends SubsystemBase{

    /*                  Meathods:
     *
     * setColourHSV(int hue, int saturation, int value);
     *          -sets colour using HSV
     * 
     * setColourRGB(Colour8Bit colour);
     *          -sets coulour using RGB
     * 
     * preset colors:
     *          setLEDMaroon
     *          setLEDRed
     *          setLEDBlue
     *          setLEDWhite
     *          setLEDoff
     * 
     * pulseColour(Color8Bit baseColour, Color8Bit pulseColour, int width, int speed);
     *          -pules a "pulseColour" ontop of a "baseColour" 
     *          -has a width of "width" LED's 
     *          -and moves a a speed of "speed" LED's / for loop iteration
     * 
     */


    AddressableLED m_Led= new AddressableLED(LedConstants.kLedPort); //define port
    AddressableLEDBuffer m_LedBuffer = new AddressableLEDBuffer(LedConstants.kLedLength); //define leng
 
    public Led() {
        m_Led.setLength(m_LedBuffer.getLength());
        m_Led.setData(m_LedBuffer);
        m_Led.start();
    }

    //changes colour of all Led's to a certan RGB
    public void setColourRGB(Color8Bit colour) { 
        for (int i = 0; i < m_LedBuffer.getLength(); i++) {
            m_LedBuffer.setRGB(i , colour.red, colour.green, colour.blue);
        }
        m_Led.setData(m_LedBuffer); 
    }

    
    public void setLEDRed() {
        for (int i = 0; i < m_LedBuffer.getLength(); i++) { //:)
            m_LedBuffer.setRGB(i , 255, 0, 0);
        }
        m_Led.setData(m_LedBuffer); 
    }

    public void setLEDBlue() {
        for (int i = 0; i < m_LedBuffer.getLength(); i++) {
            m_LedBuffer.setRGB(i , 0, 0, 255);
        }
        m_Led.setData(m_LedBuffer); 
    }

    public void setLEDMaroon() {
        for (int i = 0; i < m_LedBuffer.getLength(); i++) {
            m_LedBuffer.setRGB(i , 128, 0, 0);
        }
        m_Led.setData(m_LedBuffer); 
    }

    public void setLEDWhite() {
        for (int i = 0; i < m_LedBuffer.getLength(); i++) {
            m_LedBuffer.setRGB(i , 255, 255, 255);
        }
        m_Led.setData(m_LedBuffer); 
    }

    public void setLEDoff() {
        for (int i = 0; i < m_LedBuffer.getLength(); i++) {
            m_LedBuffer.setRGB(i , 0, 0, 0);
        }
        m_Led.setData(m_LedBuffer); 
    }
    
    //changes colour of all Led's to a certan HSV
    public void setColourHSV(int hue, int saturation, int value) { 
        for (int i = 0; i < m_LedBuffer.getLength(); i++) {
            m_LedBuffer.setHSV(i, hue, saturation, value);
            }
        m_Led.setData(m_LedBuffer); 
    }

    //Makes a pulse of a certian colour pulse across it 
    //Takes in an arrays in form {red value, green value, blue value}
    public void pulseColour(Color8Bit baseColour, Color8Bit pulseColour, int width, int speed) {
        int leadingPixel = 0; 

        //change colour to base colour 
        for (int i = 0; i < m_LedBuffer.getLength(); i++) {
            m_LedBuffer.setRGB(i, baseColour.red, baseColour.blue, baseColour.green);
            }
         m_Led.setData(m_LedBuffer); 
    
        //doing the pulse 
        int startingPixel = leadingPixel - (width / 2); 
        for (int i = startingPixel; i < width; i++) {
        if (leadingPixel > m_LedBuffer.getLength()) {break;}//:)
        m_LedBuffer.setRGB(i, pulseColour.red, pulseColour.blue, pulseColour.green);
        }

        //iterate 
        startingPixel += speed; 
    }

    public void constantRGB() {

    }


}







