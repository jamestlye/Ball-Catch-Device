## About

The Ball Catch Device is a complementary project with the Eye Tracking Goggles. The goal is to add in hand-eye coordination tests to see how well do the eyes perform relative to the hands.

## Hardware

* [Adafruit Feather 32u4 Bluefruit LE](https://www.adafruit.com/product/2829)
* [Flex Sensor](https://www.sparkfun.com/products/10264)
* Flexible TPE 

## LEAST SQUARE!!

```cpp
  //LEAST SQUARE APPROXIMATION (to find line of best fit's slope)(unit: volt/ms) 
  sum_x = sum_y = sum_x2 = sum_xy = 0;
  
  for (int k = 0; k < 15; k++){

    clippedTime = past15Times[k] - past15Times[0];
    
    sum_x += clippedTime;
    sum_y += past15Avg[k];
    sum_x2 += clippedTime*clippedTime;
    sum_xy += past15Avg[k]*clippedTime;
  }

  slope = (15.0 * sum_xy - sum_x * sum_y)/(15.0 * (double)(sum_x2) - (double)(sum_x * sum_x));
```

## Notes

Version 3 is Generation 2 - Version 1 (Upgraded from Generation 1 - Version 2, which was a different set of hardware). For the actual code that was sent to the Arduino, go to [bleuart_datamode.ino](https://github.com/jamestlye/Ball-Catch-Device/tree/master/Catch%20Sensor%20V3.0/bleuart_datamode)

Other codes are test/prototype codes.

