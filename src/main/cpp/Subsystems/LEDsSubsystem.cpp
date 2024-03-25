#include "Subsystems/LEDsSubsystem.h"
#include "Subsystems/IndexerSubsytem.h"

LEDsSubsystem::LEDsSubsystem(IndexerSubsystem *indexer) : COMETS3357::Subsystem{"LEDs"}
{
    indexerSubsytem = indexer;
        m_led.SetLength(50);
        m_led.SetData(ledBuffer);
    m_led.Start();
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
    frameSub = table->GetIntegerTopic("Frames").Subscribe(0);

 
}

void LEDsSubsystem::Periodic() {

    for (int i = 0; i < 50; i++) {
                ledBuffer[i].SetRGB(0, 255, 0);
            }
            m_led.SetData(ledBuffer);

    enabled = frc::DriverStation::IsEnabled();
    comms = frc::DriverStation::IsDSAttached();

    if ((double)wpi::math::MathSharedStore::GetTimestamp() > lastTestTimestamp)
    {
        lastTestTimestamp += 1;
        frc::SmartDashboard::PutBoolean("LEDPiStatus", frameSub.Get() != lastFrame);
        lastFrame = frameSub.Get();
    }

    if (!enabled) {
        
        if (!comms)
        {
            writeLEDs(0,255,0); // red 
            for (int i = 0; i < 50; i++) {
                ledBuffer[i].SetRGB(0, 255, 0);
            }
            m_led.SetData(ledBuffer);
        }   
        else if (gyroZero)
        {
            writeLEDs(50,255,0); // orange
            for (int i = 0; i < 50; i++) {
                ledBuffer[i].SetRGB(50, 255, 0);
            }
            m_led.SetData(ledBuffer);
        } 
        else 
        {
            writeLEDs(255,0,0); // green
            for (int i = 0; i < 50; i++) {
                ledBuffer[i].SetRGB(255, 0, 0);
            }
            m_led.SetData(ledBuffer);
        } 
    }


    if (enabled) {
        std::string mode = nt::NetworkTableInstance::GetDefault().GetTable("mode")->GetEntry("mode").GetString("mode"); 
        if (ampSignal)
        {
            writeLEDs(255,255,0); // yellow // signal to activate amp
            for (int i = 0; i < 50; i++) {
                ledBuffer[i].SetRGB(255, 255, 0);
            }
            m_led.SetData(ledBuffer);
        } 
        else if (hpSignal)
        {
            writeLEDs(0,255,255); // purple // signal to the human player // need to make flashing
            for (int i = 0; i < 50; i++) {
                ledBuffer[i].SetRGB(0, 255, 255);
            }
            m_led.SetData(ledBuffer);
        }
        else if (mode == "XBOXManual") 
        {
            writeLEDs(128,0,128); // teal // manual mode
            for (int i = 0; i < 50; i++) {
                ledBuffer[i].SetRGB(128, 0, 129);
            }
            m_led.SetData(ledBuffer);
        } 
        else if (!indexerSubsytem->IsDetected()) 
        {
            writeLEDs(255,0,0); // green // game piece detection
            for (int i = 0; i < 50; i++) {
                ledBuffer[i].SetRGB(255, 0, 0);
            }
            m_led.SetData(ledBuffer);
        } 
        else
        {
            writeLEDs(50,255,0); // orange // default mode // semi-auto
            for (int i = 0; i < 50; i++) {
                ledBuffer[i].SetRGB(50, 255, 0);
            }
            m_led.SetData(ledBuffer);
        } 
    }
}
