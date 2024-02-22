#include "Subsystems/LEDsSubsystem.h"


LEDsSubsystem::LEDsSubsystem() : COMETS3357::Subsystem("LEDsSubsystem") {


}

void LEDsSubsystem::writeLEDs(int g, int r, int b) {
    greenPub.Set(g);
    redPub.Set(r);
    bluePub.Set(b);

    
}

void LEDsSubsystem::Initialize()
{

    

   
    greenPub = table->GetIntegerTopic("gValue").Publish();
    redPub = table->GetIntegerTopic("rValue").Publish();
    bluePub = table->GetIntegerTopic("bValue").Publish();

}

void LEDsSubsystem::Periodic() {

}
