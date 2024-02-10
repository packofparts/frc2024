public class LED {
    public static void main(String[] args){
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
   // Sets the specified LED to the RGB values for red
   
   
   

   if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            m_ledBuffer.setRGB(i, 255, 0, 0);
            return;
        } else if (Math.abs(desiredState.speedMetersPerSecond) <= 1.788) {
            stop();
            m_ledBuffer.setRGB(i, 128, 128, 0);
            return;
        } else if (Math.abs(desiredState.speedMetersPerSecond) >= 1.788) {
            stop();
            m_ledBuffer.setRGB(i, 0, 255, 0);
            return;
        }
}

m_led.setData(m_ledBuffer);
    }
}
