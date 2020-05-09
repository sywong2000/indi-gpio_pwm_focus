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
#include <pigpiod_if2.h>


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
        virtual IPState MoveFocuser(FocusDirection dir, int speed, uint16_t duration) override;
        virtual bool AbortFocuser() override;
        virtual bool ReverseFocuser(bool enabled) override;


    private:

        INumber gpioN[5];
        INumberVectorProperty gpioNP;
        bool isReversed;
        bool isMoving;
        int pi_gpio;
};

#endif
