package frc.robot.utilities;

import edu.wpi.first.wpilibj.util.Color;

public class LEDAnimations {
  private int count = 0;
  private int maxCount = 200000;

  public LEDAnimations() {}

  public LEDAnimations(int maxCount) {
    this.maxCount = maxCount;
  }

  // Display the section in one color
  public void monotone(LEDSubStrip section, Color color) {
    for (int i = 0; i < section.getLength(); i++) {
      section.setLED(i, color);
    }
  }

  // Alternate colors up the strip
  public void alternate(LEDSubStrip section, Color color1, Color color2) {
    for (int i = 0; i < section.getLength(); i++) {
      if (i % 2 == 0) {
        section.setLED(i, color1);

      } else {
        section.setLED(i, color2);
      }
    }
  }

  // Flash a color on and off
  public void flashing(LEDSubStrip section, Color color, int interval) {
    for (int i = 0; i < section.getLength(); i++) {
      if ((count % (2 * interval) < interval)) {
        section.setLED(i, color);
      } else {
        section.setRGB(i, 0, 0, 0);
      }
    }
  }

  // Run a color up the strip like an animated runway
  // If anyone is looking at this I gave up at 2:23 AM on 12/20/23
  // I think I may have a concussion from excessive coughing and this little funtion hurts my brain
  public void runway(LEDSubStrip section, Color color, int interval) {
    for (int i = 0; i < section.getLength(); i++) {
      // finds where is the sequence the "chasing" LED is
      if ((i + count % 4) == interval) {
        section.setLED(i, color);
      } else {
        section.setRGB(i, 0, 0, 0);
      }
    }
  }

  public void update() {
    count = (count + 1) % maxCount;
  }
}
