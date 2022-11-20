// main.cpp
#include "pch.h"
#include <concrt.h>

#define BUFFER_SIZE 4

using namespace winrt;
using namespace Windows::Gaming::Input;


int main()
{
    winrt::init_apartment();
    std::vector<RacingWheel> myRacingWheels;
    std::vector<Gamepad> myGamepads;
    concurrency::critical_section myLock_1{};
    concurrency::critical_section myLock_2{};
    
    std::cout << "scanning controllers..." << std::endl;
    while (RacingWheel::RacingWheels().Size() < 1 && Gamepad::Gamepads().Size() < 1);
    std::cout << "Controller detected:" << std::endl;
    std::cout << "[Gamepads]: " << Gamepad::Gamepads().Size() << std::endl;
    std::cout << "[RacingWheels]: " << RacingWheel::RacingWheels().Size() << std::endl;


    for (auto const& gamepad : Gamepad::Gamepads()) {
        // Test whether the gamepad is already in myGamepads; if it isn't, add it.
        concurrency::critical_section::scoped_lock lock{ myLock_1 };
        auto it{ std::find(begin(myGamepads), end(myGamepads), gamepad) };

        if (it == end(myGamepads)) {
            // This code assumes that you're interested in all gamepads.
            myGamepads.push_back(gamepad);
        }
    }

    for (auto const& racingWheel : RacingWheel::RacingWheels()) {
        // Test whether the gamepad is already in myGamepads; if it isn't, add it.
        concurrency::critical_section::scoped_lock lock{ myLock_2 };
        auto it{ std::find(begin(myRacingWheels), end(myRacingWheels), racingWheel) };

        if (it == end(myRacingWheels)){
            // This code assumes that you're interested in all gamepads.
            myRacingWheels.push_back(racingWheel);
        }
    }

    

    SerialPort* sp = new SerialPort("COM5");    // serial communication init, adjust as needed
    RacingWheel* racingWheel = nullptr;
    Gamepad* gamepad = nullptr;
    GamepadReading gamepadReading;
    RacingWheelReading racingWheelReading;

    if (myRacingWheels.size()) racingWheel = &myRacingWheels[0];
    if (myGamepads.size()) gamepad = &myGamepads[0];

    if (racingWheel && racingWheel->WheelMotor()) {
        //ForceFeedback::ForceFeedbackMotor motor = racingWheel->WheelMotor();
        //motor.MasterGain(0.90);
        //
        racingWheel->WheelMotor().MasterGain(0.50);
        //racingWheel->WheelMotor().StopAllEffects();
        //racingWheel->WheelMotor().TryDisableAsync();
    }


    char buffer[BUFFER_SIZE];
    int counter = 0;
    char* bytePt;
    while (true) {
       
        if (racingWheel) {

            //if (racingWheel->WheelMotor()) racingWheel->WheelMotor().MasterGain(0.0);

            racingWheelReading = racingWheel->GetCurrentReading();
            // conveting double to 8-bit int
            buffer[0] = char(racingWheelReading.Wheel * 127); // signed value
            buffer[1] = char(racingWheelReading.Throttle * 255); // unsigned value
            buffer[2] = char(racingWheelReading.Brake * 255);

            // reading.button is a 32-bit enum, only sending the least significante byte, 
            //  including the information of gear changing and d-pad 
            bytePt = (char*)&racingWheelReading.Buttons;
            buffer[3] = bytePt[0];
            /*printf("size=%d ", sizeof(racingWheelReading));
            printf("wheel=%f    ", racingWheelReading.Wheel);
            printf("throttle=%f    ", racingWheelReading.Throttle);
            printf("brake=%f    ", racingWheelReading.Brake);
            printf("buttons=%d    \n", bytePt[0]);*/
        }

        if (gamepad) {
            gamepadReading = gamepad->GetCurrentReading();           

            // conveting double to 8-bit int
            buffer[0] = char(gamepadReading.LeftThumbstickX * 127); // signed value
            buffer[1] = char(gamepadReading.RightTrigger * 255); // unsigned value
            buffer[2] = char(gamepadReading.LeftTrigger * 255);
            bytePt = (char*)&gamepadReading.Buttons;

            buffer[3] = bytePt[1] >> 2;
           
            /*printf("size=%d ", sizeof(gamepadReading));
            printf("size=%d ", myGamepads.size());
            printf("Wheel=%f    ", gamepadReading.LeftThumbstickX);
            printf("throttle=%f    ", gamepadReading.RightTrigger);
            printf("brake=%f    ", gamepadReading.LeftTrigger);
            printf("buttons=%d    \n", bytePt[1]>>2);*/
        }


        // sending data over Serial Communication
        if (sp->isConnected() && !(counter % 40)) {
            //sp->writeSerialPort((const char*)&reading, sizeof(RacingWheelReading));
            sp->writeSerialPort((const char*)buffer, BUFFER_SIZE);
            printf("size=%d ", sizeof(RacingWheelReading));
            printf("Wheel=%d    ", buffer[0]);
            printf("throttle=%d    ",(unsigned char)buffer[1]);
            printf("brake=%d    ", buffer[2]);
            printf("buttons=%d\n", buffer[3]);
        }
        counter++;
    }
    
}