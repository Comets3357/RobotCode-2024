#include "COMETS3357/Subsystems/LED/LED.h"


AddressableLED::AddressableLED()
{
    m_led.SetLength(kLength);

    for (int i = 0; i < kLength; i++) 
    {
   comets_ledBuffer[i].SetRGB(0,0,255);
    }
    



    m_led.SetData(comets_ledBuffer);
    m_led.Start();
}