package frc.robot.subsystems;
import org.opencv.objdetect.BaseCascadeClassifier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;; 



public class Led extends SubsystemBase{


    AddressableLED m_Led= new AddressableLED(LedConstants.kLedPort); //define port
    AddressableLEDBuffer m_LedBuffer = new AddressableLEDBuffer(LedConstants.kLedLength); //define leng
 
    public Led() {
        m_Led.setLength(m_LedBuffer.getLength());
        m_Led.setData(m_LedBuffer);
        m_Led.start();
    }

    //changes colour of all Led's to a certan RGB
    public Command setColourRGB(Color colour) { 
        return runOnce(() -> { 
            for (int i = 0; i < m_LedBuffer.getLength(); i++) {
            m_LedBuffer.setRGB(i , (int)(colour.red * 255), (int)(colour.green * 255), (int)(colour.blue * 255));
          }
        }); 
    }

    public Command setLEDRed() {
       return setColourRGB(new Color(255,0,0));
    }

    public Command setLEDBlue() {
        return setColourRGB(new Color(0,0,255));
    }

    public Command setLEDWhite() {
       return setColourRGB(new Color(255,255,255));
    }

    public Command setLEDoff() {
        return setColourRGB(new Color(0,0,0));
    }
    
    //changes colour of all Led's to a certan HSV
    public void setColourHSV(int hue, int saturation, int value) { 
        setColourRGB(Color.fromHSV(hue, saturation, value));
    }

    //TODO: may need to make this a command: 
    //Makes a pulse of a certian colour pulse across it 
    public void pulseColour(Color baseColour, Color pulseColour, int width, int speed) {     
        int leadingPixel = 0; 
        //change colour to base colour 
        setColourRGB(baseColour); 
    
        //doing the pulse 
        int startingPixel = leadingPixel - (width / 2); 
        for (int i = startingPixel; i < width; i++) {
             if (leadingPixel > m_LedBuffer.getLength()) {break;}
             m_LedBuffer.setRGB(i, (int)(pulseColour.red * 255), (int)(pulseColour.green * 255), (int)(pulseColour.blue * 255));
        }

        //iterate 
        startingPixel += speed; 
    }

    public Command runPulseColour(Color baseColour, Color pulseColor, int width, int speed) {
        return runOnce(
        () -> {
          pulseColour(baseColour, pulseColor, width, speed);
          m_Led.setData(m_LedBuffer);
        });
    }

      public Command dropDisk(){
    return setColourRGB(new Color(0,255,100));
  
  }
  public Command coop(){
     return setColourRGB(new Color(255,255,0));
    
  }
  public Command ampSpeaker(){
    return setColourRGB(new Color(128,0,128));
    
  }
  public Command readyToAmp(){
     return setColourRGB(new Color(255, 192, 203));
     //change color later bc alliance problems
  }
  public Command readyToSpeaker(){
    return setColourRGB(new Color(0, 255, 0));
    
  }   
}







