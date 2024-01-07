#pragma once

#include <frc/AddressableLED.h>


namespace COMETS3357
{

    struct LED
    {

    };

    class AddressableLED : public frc::AddressableLED
    {
    public:
        AddressableLED();
        
        void frc::AddressableLED::SetData(std::initializer_list<LEDData>ledData)	
        void frc::AddressableLED::LEDData::SetRGB(int r, int g, int b);

    private:


        static constexpr int kLength = 100;
        
        frc::AddressableLED comets_led{};
        std::array<frc::AddressableLED::LEDData, kLength> comets_ledBuffer;




    };

};
