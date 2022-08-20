package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase{
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    private AddressableLEDSim m_led_sim;
    // Store what the last hue of the first pixel is
    private int m_rainbowFirstPixelHue;
    private int m_mardiGrasStartPoint;

    //Shooter constructor - creates a shooter in robot memory
    public LEDs(int pwmPort, int bufferSize){
        m_led = new AddressableLED(pwmPort);
        m_ledBuffer = new AddressableLEDBuffer(bufferSize);

        m_led_sim = new AddressableLEDSim(m_led);
        m_led_sim.setLength(bufferSize);

        m_led.setLength(bufferSize);
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void update() {
        m_led.setData(m_ledBuffer);
    }

    public void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 255);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        update();
    }

    public void mardiGras() {
      int val;
      if(Timer.getFPGATimestamp()*100%20>18){
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          final var hue = (int)(30*Math.exp((m_mardiGrasStartPoint+i)%3)-10*(Math.exp((m_mardiGrasStartPoint+i)%3)-1));
          if (hue>100){val = 200;}
          else {val = 255;}
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, val);
        }
      }
      // Increase by to make the colors "move"
      m_mardiGrasStartPoint += 1;
      // Check bounds
      m_mardiGrasStartPoint %= 3;
      update();
    }

    public void solid(int hue) {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 255);
        }
        update();
    }

    public void teamColor(String isRed) {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Set the value
          int hue = (isRed == "Red") ? 0 : 120;
          m_ledBuffer.setHSV(i, hue, 255, 255);
        }
        update();
    }

    public void stripeRB() {
      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Set the value
        int hue = 120*(i%2);
        m_ledBuffer.setHSV(i, hue, 255, 255);
      }
      update();
    }
}
