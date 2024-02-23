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
    while (!enabled) {
        if (gyroZero)
        {
            LEDsSubsystem::writeLEDs(8,255,0); // orange
            greenPub = table->GetIntegerTopic("gValue").Publish();
            redPub = table->GetIntegerTopic("rValue").Publish();
            bluePub = table->GetIntegerTopic("bValue").Publish();
        } 
        else if (comms)
        {
            LEDsSubsystem::writeLEDs(255,0,0); // green
            greenPub = table->GetIntegerTopic("gValue").Publish();
            redPub = table->GetIntegerTopic("rValue").Publish();
            bluePub = table->GetIntegerTopic("bValue").Publish();
        } 
        else 
        {
            LEDsSubsystem::writeLEDs(0,255,0); // red 
            greenPub = table->GetIntegerTopic("gValue").Publish();
            redPub = table->GetIntegerTopic("rValue").Publish();
            bluePub = table->GetIntegerTopic("bValue").Publish();
        } 
        
        
        
        
    }


    while (enabled) {
        std::string mode = nt::NetworkTableInstance::GetDefault().GetTable("mode")->GetEntry("mode").GetString("mode"); 
        if (ampSignal)
        {
            LEDsSubsystem::writeLEDs(255,255,0); // yellow // signal to activate amp
            greenPub = table->GetIntegerTopic("gValue").Publish();
            redPub = table->GetIntegerTopic("rValue").Publish();
            bluePub = table->GetIntegerTopic("bValue").Publish();
        } 
        else if (hpSignal)
        {
            LEDsSubsystem::writeLEDs(0,255,255); // purple // signal to the human player // need to make flashing
            greenPub = table->GetIntegerTopic("gValue").Publish();
            redPub = table->GetIntegerTopic("rValue").Publish();
            bluePub = table->GetIntegerTopic("bValue").Publish();
        }
        else if (mode == "XBOXManual") 
        {
            LEDsSubsystem::writeLEDs(255,0,255); // teal // manual mode
            greenPub = table->GetIntegerTopic("gValue").Publish();
            redPub = table->GetIntegerTopic("rValue").Publish();
            bluePub = table->GetIntegerTopic("bValue").Publish();
        } 
        // else if (IndexerSubsystem::IsDetected) 
        // {
        //     LEDsSubsystem::writeLEDs(255,0,0); // green // game piece detection
        //     greenPub = table->GetIntegerTopic("gValue").Publish();
        //     redPub = table->GetIntegerTopic("rValue").Publish();
        //     bluePub = table->GetIntegerTopic("bValue").Publish();
        // } 
        else 
        {
            LEDsSubsystem::writeLEDs(8,255,0); // orange // default mode // semi-auto
            greenPub = table->GetIntegerTopic("gValue").Publish();
            redPub = table->GetIntegerTopic("rValue").Publish();
            bluePub = table->GetIntegerTopic("bValue").Publish();
        } 
    }
}
