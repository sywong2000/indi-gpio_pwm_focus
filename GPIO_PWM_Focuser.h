/*
  INDI Driver for GPIO PWM

  Copyright (C) 2020 Stephen Wong

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef GPIO_PWM_Focuser_H
#define GPIO_PWM_Focuser_H

#include <string>

#include <indidevapi.h>
#include <indicom.h>
#include <indifocuser.h>
#include <pigpio.h>


using namespace std;

#define GPIO_PWM_Focuser_STEP_SIZE      32
#define GPIO_PWM_Focuser_ERROR_BUFFER   1024


class GPIO_PWM_Focuser : public INDI::Focuser
{

    public:

        GPIO_PWM_Focuser();

        const char *getDefaultName() override;
        virtual bool initProperties() override;
        virtual bool updateProperties() override;
        virtual bool saveConfigItems(FILE *fp) override;
        virtual bool ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n) override;
        virtual bool SetFocuserSpeed(int speed) override;
        virtual bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n) override;
        virtual bool Connect() override;
        virtual bool Disconnect() override;

    protected:
        //virtual bool Handshake() override;
        //virtual void TimerHit() override;
//        virtual bool SyncFocuser(uint32_t ticks) override;

//        virtual IPState MoveAbsFocuser(uint32_t ticks) override;
//        virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
        virtual IPState MoveFocuser(FocusDirection dir, int speed, uint16_t duration) override;
        virtual bool AbortFocuser() override;
        virtual bool ReverseFocuser(bool enabled) override;


    private:

//        INumber WeatherN[3];
//        INumberVectorProperty WeatherNP;

//        ISwitch ParkS[2];
//        ISwitchVectorProperty ParkSP;

//        ISwitch StatusS[3];
//        ISwitchVectorProperty StatusSP;
//        INumber GPIO_pwm_pin_num;
//        INumber GPIO_A01_pin_num;
//        INumber GPIO_A02_pin_num;
//        INumber GPIO_Enable_pin_num;
//        INumber FocuserSpeed;

        INumber gpioN[5];
        INumberVectorProperty gpioNP;
        bool isReversed;

//        ISwitch gpioFeed33vS[1];
//        ISwitchVectorProperty gpioFeed33vSP;
//        enum { GPIO_FEED_33V_YES, GPIO_FEED_33V_NO};

//        INumber FocuserN[1];
//        INumberVectorProperty FocuserNP;

        //INumber SetBacklashN[1];
        //INumberVectorProperty SetBacklashNP;

//        unsigned char calculate_checksum(GPIO_PWM_FocuserCommand c);
//        bool send_command(char k, uint32_t l = 0, unsigned char addr = 0);
//        bool read_response();
//        bool dispatch_command(char k, uint32_t l = 0, unsigned char addr = 0);

//        bool getTemperature();
//        bool getStatus();
//        bool getPosition();
//        bool getMaxPosition();
//        bool setPosition(int32_t position);
//        bool setSync(uint32_t position = 0);
//        bool setPark();

       // Variables
//            int PWMPinNum;
//            int A01PinNum;
//            int A02PinNum;
//            int EnablePinNum;
//            int speed;

//        float currentTemperature;
//        float currentHumidity;
//        int32_t currentPosition;
//        int32_t currentMaxPosition;
//        bool isAbsolute;
        bool isMoving;
//        unsigned char isParked;
//        bool isVcc12V;
//        GPIO_PWM_FocuserCommand currentResponse;
};

#endif
