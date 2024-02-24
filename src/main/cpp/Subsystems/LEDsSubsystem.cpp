#include "Subsystems/LEDsSubsystem.h"


LEDsSubsystem::LEDsSubsystem() : COMETS3357::Subsystem("LEDsSubsystem") {


}


void LEDsSubsystem::writeLEDs(int g, int r, int b) {
    redPub.Set(r);
    greenPub.Set(g);
    bluePub.Set(b);

    
}

void LEDsSubsystem::Initialize()
{
    redPub = table->GetIntegerTopic("rValue").Publish();
    greenPub = table->GetIntegerTopic("gValue").Publish();
    bluePub = table->GetIntegerTopic("bValue").Publish();

}

void LEDsSubsystem::Periodic() {
    while (!enabled) {
        if (gyroZero)
        {
            LEDsSubsystem::writeLEDs(100,255,0); // orange
            redPub = table->GetIntegerTopic("rValue").Publish();
            greenPub = table->GetIntegerTopic("gValue").Publish();
            bluePub = table->GetIntegerTopic("bValue").Publish();
        } 
        else if (comms)
        {
            LEDsSubsystem::writeLEDs(255,0,0); // green
            redPub = table->GetIntegerTopic("rValue").Publish();
            greenPub = table->GetIntegerTopic("gValue").Publish();
            bluePub = table->GetIntegerTopic("bValue").Publish();
        }   
        else 
        {
            LEDsSubsystem::writeLEDs(0,255,0); // red 
            redPub = table->GetIntegerTopic("rValue").Publish();
            greenPub = table->GetIntegerTopic("gValue").Publish();
            bluePub = table->GetIntegerTopic("bValue").Publish();
        } 
        
        
        
        
    }


    while (enabled) {
        std::string mode = nt::NetworkTableInstance::GetDefault().GetTable("mode")->GetEntry("mode").GetString("mode"); 
        if (ampSignal)
        {
            LEDsSubsystem::writeLEDs(255,255,0); // yellow // signal to activate amp
            redPub = table->GetIntegerTopic("rValue").Publish();
            greenPub = table->GetIntegerTopic("gValue").Publish();
            bluePub = table->GetIntegerTopic("bValue").Publish();
        } 
        else if (hpSignal)
        {
            LEDsSubsystem::writeLEDs(0,255,255); // purple // signal to the human player // need to make flashing
            redPub = table->GetIntegerTopic("rValue").Publish();
            greenPub = table->GetIntegerTopic("gValue").Publish();
            bluePub = table->GetIntegerTopic("bValue").Publish();
        }
        else if (mode == "XBOXManual") 
        {
            LEDsSubsystem::writeLEDs(128,0,128); // teal // manual mode
            redPub = table->GetIntegerTopic("rValue").Publish();
            greenPub = table->GetIntegerTopic("gValue").Publish();
            bluePub = table->GetIntegerTopic("bValue").Publish();
        } 
        else if (IndexerSubsystem::IsDetected) 
        {
            LEDsSubsystem::writeLEDs(255,0,0); // green // game piece detection
            redPub = table->GetIntegerTopic("rValue").Publish();
            greenPub = table->GetIntegerTopic("gValue").Publish();
            bluePub = table->GetIntegerTopic("bValue").Publish();
        } 
        else
        {
            LEDsSubsystem::writeLEDs(100,255,0); // orange // default mode // semi-auto
            redPub = table->GetIntegerTopic("rValue").Publish();
            greenPub = table->GetIntegerTopic("gValue").Publish();
            bluePub = table->GetIntegerTopic("bValue").Publish();
        } 
    }
}
