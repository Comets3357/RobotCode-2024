#include "COMETS3357/Subsystems/LED/LED.h"


AddressableLED::AddressableLED()
{
    for (int i = 0; i < kLength; i++) 
    {
   comets_ledBuffer[i].SetRGB(0,0,255);
    }
    



    m_led.SetLength(kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
}