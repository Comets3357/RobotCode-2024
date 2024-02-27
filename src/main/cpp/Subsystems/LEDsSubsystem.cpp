#include "Subsystems/LEDsSubsystem.h"
#include "Subsystems/IndexerSubsytem.h"

LEDsSubsystem::LEDsSubsystem(IndexerSubsystem *indexer) : COMETS3357::Subsystem{"LEDs"}
{
    indexerSubsytem = indexer;
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

    enabled = frc::DriverStation::IsEnabled();
    comms = frc::DriverStation::IsDSAttached();

    if (!enabled) {
        
        if (!comms)
        {
            writeLEDs(0,255,0); // red 
        }   
        else if (gyroZero)
        {
            writeLEDs(50,255,0); // orange
        } 
        else 
        {
            writeLEDs(255,0,0); // green
        } 
    }


    if (enabled) {
        std::string mode = nt::NetworkTableInstance::GetDefault().GetTable("mode")->GetEntry("mode").GetString("mode"); 
        if (ampSignal)
        {
            writeLEDs(255,255,0); // yellow // signal to activate amp
        } 
        else if (hpSignal)
        {
            writeLEDs(0,255,255); // purple // signal to the human player // need to make flashing
        }
        else if (mode == "XBOXManual") 
        {
            writeLEDs(128,0,128); // teal // manual mode
        } 
        else if (!indexerSubsytem->IsDetected()) 
        {
            writeLEDs(255,0,0); // green // game piece detection
        } 
        else
        {
            writeLEDs(50,255,0); // orange // default mode // semi-auto
        } 
    }
}
