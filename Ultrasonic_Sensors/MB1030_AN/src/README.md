To summarise:
    More understanding is required to get a proper raw ADC value reading from the ESP32 which properly corresponds with the voltage reading being taken. 
    Currently the Sensor returns a voltage reading which is accurate with the distance that is expected from the voltage, however the voltage returned is between 0V to 3.3V with a corresponding distance of 0cm to 1300cm (13m). This is excessive as the sensor is not able to accurately measure anything beyond 6m. With most objects being located between 20cm and 300cm, which correspond to voltage readings between 50.8mV and 761.4mV, the need is to more accurately measure the voltage between these two ranges. 
    However the ESP32 is struggling to read voltages in this range as they are so low that it jumps around different voltages due to noise which can drastically make the estimated distance very different (and wrong). 
    
    A few siggestions:
    - Increase the VCC, as this would increase the retuned voltage (20cm and 300cm is now 77mV and 1.15V), a higher voltage will be easier for the esp32 to read, however the risk is that the ADC may get damaged if a voltage higher than 3.3V is returned (3.3V would be 8.6m, Hallway is longer therefore there is a risk of damaging the ESP32)
    - Get a better ADC chip. The risk here is that it takes away time and adds complexity for our 2nd mapping system. Therefore I would suggest to dismiss and not try to make this work, as the time to make stuff work is better spent elsewhere (mapping system 1, drone flight). 



Notes:

  analogSetPinAttenuation(anPin, ADC_0db); // 0dB -> best sensitivity (~1.1V FS); This is not accurate for a 3.3V supply, however seems to work better for objects that are 40-60cm away. Otherwise objects are read as closer for onjects that are closer (e.g. 30cm is read as 10cm); or further for objects that are further away (80cm is read as 300cm). 
  analogSetWidth(12);                      // 12-bit -> 0..4095


// More testing is required. The returned raw reading does not seems to go higher than 1.2V. About 10cm on the oscilliscope is reading at 50mV.


 // collect raw data for different attenuations levels at different distances. Then can attempt to find a relationship between raw value and distance.

 // Also maybe look at getting a better ADC chip. This would mean the project is getting a bit too complex than required. This is still the 2nd mapping option, so does not need to be perfect. I think PW will be good enough. 
 // The voltage reading on the osciliscope seems to be right with calculation, where the voltage reading can be converted to a distance, e.g. 77mV ~ 30cm. It just seems the ADC on the ESP32 is struglling or something else is causing issues. For example when the attenuation is set at 11dB (ADC_11db), each step is 0.805mV per ADC which should be good enough, so IDK why it is not working. 