/*
  INDI Driver for gpioPWM

  Copyright (C) 2016 Piotr Dlugosz

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

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <memory>
#include <indicom.h>
#include <pigpio.h>

#include "GPIO_PWM_Focuser.h"
#include "config.h"



//#define PARK_PARK 0
//#define PARK_UNPARK 1

// We declare an auto pointer to gpioPWM.
static std::unique_ptr<GPIO_PWM_Focuser> gpioPWMFocuser(new GPIO_PWM_Focuser());

void ISPoll(void *p);

void ISGetProperties(const char *dev)
{
    gpioPWMFocuser->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
    gpioPWMFocuser->ISNewSwitch(dev, name, states, names, num);
}

void ISNewText(	const char *dev, const char *name, char *texts[], char *names[], int num)
{
    gpioPWMFocuser->ISNewText(dev, name, texts, names, num);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
    gpioPWMFocuser->ISNewNumber(dev, name, values, names, num);
}

void ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}

void ISSnoopDevice (XMLEle *root)
{
    gpioPWMFocuser->ISSnoopDevice(root);
}

/****************************************************************
**
**
*****************************************************************/

GPIO_PWM_Focuser::GPIO_PWM_Focuser()
{
    //FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT | FOCUSER_CAN_SYNC);
    FI::SetCapability(FOCUSER_CAN_ABORT |FOCUSER_CAN_REVERSE|FOCUSER_HAS_VARIABLE_SPEED);
    //wiringPiSetupGpio () ;

//    isAbsolute = false;
//    isMoving = false;
//    isParked = 0;
//    isVcc12V = false;
    setSupportedConnections(CONNECTION_NONE);
    setVersion(GPIO_PWM_VERSION_MAJOR, GPIO_PWM_VERSION_MINOR);

}

bool GPIO_PWM_Focuser::initProperties()
{

    INDI::Focuser::initProperties();
    try
    {
        gpioInitialise();
    }
    catch (exception ex)
    {
        LOG_ERROR("Unable to initialize GPIO. Are you using a Raspberry Pi?");
        return false;
    }


    IUFillNumber(&GPIO_pwm_pin_num,"GPIOPWMPIN","GPIO Pin Number","%d",0,40,1,5);
    IUFillNumber(&GPIO_A01_pin_num,"GPIOA01PIN","GPIO INA1 Pin Number","%d",0,40,1,6);
    IUFillNumber(&GPIO_A02_pin_num,"GPIOA02PIN","GPIO INA2 Pin Number","%d",0,40,1,13);
    IUFillNumber(&GPIO_Enable_pin_num,"GPIOENABLEPIN","GPIO Enable Pin Number","%d",0,40,1,19);
    IUFillNumber(&FocuserSpeed,"FOCUSERSPEED","Speed","%.f",0,127,1,50);

    // Default speed
    //FocusSpeedN[0].min = 0;
    //FocusSpeedN[0].max = 127;
    //FocusSpeedN[0].value = 50;
    //IUUpdateMinMax(&FocusSpeedNP);

    // Max Position
    //    IUFillNumber(&MaxPositionN[0], "MAXPOSITION", "Ticks", "%.f", 1., 500000., 1000., 300000);
    //    IUFillNumberVector(&MaxPositionNP, MaxPositionN, 1, getDeviceName(), "MAXPOSITION", "Max Absolute Position", FOCUS_SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    //    IUFillNumber(&MaxTravelN[0], "MAXTRAVEL", "Ticks", "%.f", 1., 500000., 1000., 300000.);
    //    IUFillNumberVector(&MaxTravelNP, MaxTravelN, 1, getDeviceName(), "MAXTRAVEL", "Max Relative Travel", FOCUS_SETTINGS_TAB, IP_RW, 0, IPS_IDLE );

    //    // Focus Sync
    //    IUFillSwitch(&SyncS[0], "SYNC", "Synchronize", ISS_OFF);
    //    IUFillSwitchVector(&SyncSP, SyncS, 1, getDeviceName(), "SYNC", "Synchronize", MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    // Focus Park
//    IUFillSwitch(&ParkS[PARK_PARK], "PARK", "Park", ISS_OFF);
//    IUFillSwitch(&ParkS[PARK_UNPARK], "UNPARK", "Unpark", ISS_OFF);
//    IUFillSwitchVector(&ParkSP, ParkS, 2, getDeviceName(), "PARK", "Park", MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    // Focuser temperature and humidity
//    IUFillNumber(&WeatherN[0], "TEMPERATURE", "Temperature [C]", "%6.1f", -100, 100, 0, 0);
//    IUFillNumber(&WeatherN[1], "HUMIDITY", "Humidity [%]", "%6.1f", 0, 100, 0, 0);
//    IUFillNumber(&WeatherN[2], "DEWPOINT", "Dew point [C]", "%6.1f", -100, 100, 0, 0);
//    IUFillNumberVector(&WeatherNP, WeatherN, 3, getDeviceName(), "FOCUS_WEATHER", "Weather", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // Focuser humidity
    //IUFillNumberVector(&HumidityNP, HumidityN, 1, getDeviceName(), "HUMIDITY", "Humidity", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // We init here the property we wish to "snoop" from the target device
//    IUFillSwitch(&StatusS[0], "ABSOLUTE", "Absolute", ISS_OFF);
//    IUFillSwitch(&StatusS[1], "MOVING", "Moving", ISS_OFF);
//    IUFillSwitch(&StatusS[2], "PARKED", "Parked", ISS_OFF);
//    IUFillSwitchVector(&StatusSP, StatusS, 3, getDeviceName(), "STATUS", "Status", MAIN_CONTROL_TAB, IP_RO, ISR_NOFMANY, 0, IPS_IDLE);

    //    PresetN[0].min = PresetN[1].min = PresetN[2].min = FocusAbsPosN[0].min = -MaxPositionN[0].value;
    //    PresetN[0].max = PresetN[1].max = PresetN[2].max = FocusAbsPosN[0].max = MaxPositionN[0].value;
    //    strcpy(PresetN[0].format, "%6.0f");
    //    strcpy(PresetN[1].format, "%6.0f");
    //    strcpy(PresetN[2].format, "%6.0f");
    //    PresetN[0].step = PresetN[1].step = PresetN[2].step = FocusAbsPosN[0].step = gpioPWM_STEP_SIZE;

    // Maximum position can't be changed from driver
//    FocusMaxPosNP.p = IP_RO;

//    FocusAbsPosN[0].value = 0;
//    FocusRelPosN[0].min = -FocusMaxPosN[0].max;
//    FocusRelPosN[0].max = FocusMaxPosN[0].max;
//    FocusRelPosN[0].step = gpioPWM_STEP_SIZE;
//    FocusRelPosN[0].value = 5 * gpioPWM_STEP_SIZE;

//    serialConnection->setDefaultPort("/dev/ttyACM0");
//    serialConnection->setDefaultBaudRate(Connection::Serial::B_115200);
//    setDefaultPollingPeriod(500);

    return true;
}

bool GPIO_PWM_Focuser::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
    {
//        defineNumber(&GPIO_pwm_pin_num);
//        defineNumber(&GPIO_A01_pin_num);
//        defineNumber(&GPIO_A02_pin_num);
//        defineNumber(&GPIO_Enable_pin_num);
//        defineNumber(&FocuserSpeed);
        //defineSwitch(&SyncSP);
//        defineSwitch(&ParkSP);
//        defineNumber(&WeatherNP);
//        defineSwitch(&StatusSP);
        //defineNumber(&MaxPositionNP);
        //defineNumber(&MaxTravelNP);
    }
    else
    {
        deleteProperty(GPIO_pwm_pin_num.name);
        deleteProperty(GPIO_Enable_pin_num.name);
        deleteProperty(GPIO_A01_pin_num.name);
        deleteProperty(GPIO_A02_pin_num.name);
        deleteProperty(FocuserSpeed.name);

        //deleteProperty(SyncSP.name);
//        deleteProperty(ParkSP.name);
//        deleteProperty(WeatherNP.name);
//        deleteProperty(StatusSP.name);

        //deleteProperty(MaxPositionNP.name);
        //deleteProperty(MaxTravelNP.name);
    }
    return true;
}



//void GPIO_PWM_Focuser::ISGetProperties(const char *dev)
//{
//  if(dev && strcmp(dev,getDeviceName()))
//  {
////    defineNumber(&GPIO_pwm_pin_num);
////    loadConfig(true, "GPIOPWMPIN");
////    defineNumber(&GPIO_A01_pin_num);
////    loadConfig(true, "GPIOA01PIN");
////    defineNumber(&GPIO_A02_pin_num);
////    loadConfig(true, "GPIOA02PIN");
////    defineNumber(&GPIO_Enable_pin_num);
////    loadConfig(true, "GPIOENABLEPIN");
////    defineNumber(&FocuserSpeed);
////    loadConfig(true, "FOCUSERSPEED");
//  };
//  return INDI::Focuser::ISGetProperties(dev);
//}



bool GPIO_PWM_Focuser::saveConfigItems(FILE *fp)
{
    INDI::Focuser::saveConfigItems(fp);

//    IUSaveConfigNumber(fp, &GPIO_pwm_pin_num);
//    IUSaveConfigNumber(fp, &GPIO_A01_pin_num);
//    IUSaveConfigNumber(fp, &GPIO_A02_pin_num);
//    IUSaveConfigNumber(fp, &GPIO_Enable_pin_num);
//    IUSaveConfigNumber(fp, &FocuserSpeed);

    return true;
}

bool GPIO_PWM_Focuser::ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
    return true;
}
//    if(strcmp(dev, getDeviceName()) == 0)
//    {
//        // Max Position
//        if (!strcmp(MaxPositionNP.name, name))
//        {
//            IUUpdateNumber(&MaxPositionNP, values, names, n);

//            if (MaxPositionN[0].value > 0)
//            {
//                PresetN[0].min = PresetN[1].min = PresetN[2].min = FocusAbsPosN[0].min = -MaxPositionN[0].value;;
//                PresetN[0].max = PresetN[1].max = PresetN[2].max = FocusAbsPosN[0].max = MaxPositionN[0].value;
//                IUUpdateMinMax(&FocusAbsPosNP);
//                IUUpdateMinMax(&PresetNP);
//                IDSetNumber(&FocusAbsPosNP, nullptr);

//                LOGF_DEBUG("Focuser absolute limits: min (%g) max (%g)", FocusAbsPosN[0].min, FocusAbsPosN[0].max);
//            }

//            MaxPositionNP.s = IPS_OK;
//            IDSetNumber(&MaxPositionNP, nullptr);
//            return true;
//        }


//        // Max Travel
//        if (!strcmp(MaxTravelNP.name, name))
//        {
//            IUUpdateNumber(&MaxTravelNP, values, names, n);

//            if (MaxTravelN[0].value > 0)
//            {
//                FocusRelPosN[0].min = 0;
//                FocusRelPosN[0].max = MaxTravelN[0].value;
//                IUUpdateMinMax(&FocusRelPosNP);
//                IDSetNumber(&FocusRelPosNP, nullptr);

//                LOGF_DEBUG("Focuser relative limits: min (%g) max (%g)", FocusRelPosN[0].min, FocusRelPosN[0].max);
//            }

//            MaxTravelNP.s = IPS_OK;
//            IDSetNumber(&MaxTravelNP, nullptr);
//            return true;
//        }

//    }

//    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);

//}
bool GPIO_PWM_Focuser::SetFocuserSpeed(int speed)
{
    gpioSetMode(PWM_Pin, PI_INPUT);
    gpioWrite(PWM_Pin, speed);
    //gpioTerminate();
    return true;
}

bool GPIO_PWM_Focuser::ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n)
{
//    if(strcmp(dev, getDeviceName()) == 0)
//    {
        // Park
//        if (!strcmp(ParkSP.name, name))
//        {
//            IUUpdateSwitch(&ParkSP, states, names, n);
//            int index = IUFindOnSwitchIndex(&ParkSP);
//            IUResetSwitch(&ParkSP);

//            if ( (isParked && (index == PARK_UNPARK)) || ( !isParked && (index == PARK_PARK)) )
//            {
//                LOG_INFO("Park, issuing command.");
//                if ( setPark() )
//                {
//                    //ParkSP.s = IPS_OK;
//                    FocusAbsPosNP.s = IPS_OK;
//                    IDSetNumber(&FocusAbsPosNP, nullptr);
//                }
//                else
//                    ParkSP.s = IPS_ALERT;
//            }
//            IDSetSwitch(&ParkSP, nullptr);
//            return true;
//        }
//    }

    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

//bool GPIO_PWM_Focuser::SyncFocuser(uint32_t ticks)
//{
//    return setSync(ticks);
//}

/****************************************************************
**
**
*****************************************************************/

//bool GPIO_PWM_Focuser::getTemperature()
//{
//    if ( dispatch_command('T') )
//    {
//        currentTemperature = ((short int)( (currentResponse.c << 8) | currentResponse.d )) / 10.;
//        currentHumidity = ((short int)( (currentResponse.a << 8) | currentResponse.b )) / 10.;
//    }
//    else
//        return false;
//    return true;
//}

//bool GPIO_PWM_Focuser::Handshake()
//{
//    return getStatus();
//}

//bool GPIO_PWM_Focuser::getStatus()
//{
//    LOG_DEBUG("getStatus.");
//    if ( dispatch_command('I') )
//    {
//        isMoving = ( currentResponse.d & 3 ) != 0 ? true : false;
//        //isZero = ( (currentResponse.d>>2) & 1 )  == 1;
//        isParked = (currentResponse.d>>3) & 3;
//        isVcc12V = ( (currentResponse.d>>5) & 1 ) == 1;
//    }
//    else
//        return false;

//    if ( dispatch_command('W') ) // Is absolute?
//        isAbsolute = currentResponse.d == 1 ? true : false;
//    else
//        return false;

//    return true;
//}


//bool GPIO_PWM_Focuser::getPosition()
//{
//    //int32_t pos;

//    if ( dispatch_command('P') )
//        currentPosition = (currentResponse.a << 24) | (currentResponse.b << 16) | (currentResponse.c << 8) | currentResponse.d;
//    else
//        return false;

//    return true;
//}


//bool GPIO_PWM_Focuser::setPosition( int32_t position)
//{
//    if ( dispatch_command('M', position) )
//        if ( ((currentResponse.a << 24) | (currentResponse.b << 16) | (currentResponse.c << 8) | currentResponse.d) == position )
//        {
//            LOGF_DEBUG("Moving to position %d", position);
//            return true;
//        };
//    return false;
//}

//bool GPIO_PWM_Focuser::getMaxPosition()
//{
//    if ( dispatch_command('A', 0, 3) )
//    {
//        currentMaxPosition = (currentResponse.a << 24) | (currentResponse.b << 16) | (currentResponse.c << 8) | currentResponse.d;
//        LOGF_DEBUG("getMaxPosition: %d", currentMaxPosition);
//        return true;
//    }
//    else
//      LOG_ERROR("getMaxPosition error");

//    return false;
//}


//bool GPIO_PWM_Focuser::setSync( uint32_t position)
//{
//    if ( dispatch_command('Z', position) )
//        if ( static_cast<uint32_t>((currentResponse.a << 24) | (currentResponse.b << 16) | (currentResponse.c << 8) | currentResponse.d) == position )
//        {
//            LOGF_DEBUG("Syncing to position %d", position);
//            return true;
//        };
//        LOG_ERROR("Sync failed.");
//        return false;
//}

//bool GPIO_PWM_Focuser::setPark()
//{
//    if (isAbsolute == false)
//    {
//        LOG_ERROR("Focuser is not in Absolute mode. Please sync before to allow parking.");
//        return false;
//    }

//    if ( dispatch_command('G') )
//    {
//      LOG_INFO( "Focuser park command.");
//      return true;
//    }
//    LOG_ERROR("Park failed.");
//    return false;
//}

bool GPIO_PWM_Focuser::AbortFocuser()
{
//    if ( dispatch_command('H') )
//    {
//        LOG_INFO("Focusing aborted.");
//        return true;
//    };
//    LOG_ERROR("Abort failed.");
    return false;
}



IPState GPIO_PWM_Focuser::MoveFocuser(FocusDirection dir, int speed, uint16_t duration)
{
//  gpioPWMCommand c;
//  unsigned char d = (unsigned char) (speed & 0b01111111) | (dir == FOCUS_INWARD) ? 0b10000000 : 0;

//  if ( dispatch_command('R', d) )
//  {
//    gettimeofday(&focusMoveStart,nullptr);
//    focusMoveRequest = duration/1000.0;
//    if ( read_response() )
//      if ( ( currentResponse.k == 'R' ) && (currentResponse.d == d) )
//        if (duration <= POLLMS)
//        {
//          usleep(POLLMS * 1000);
//          AbortFocuser();
//          return IPS_OK;
//        }
//        else
//          return IPS_BUSY;
//  }
  return IPS_ALERT;
}


//IPState GPIO_PWM_Focuser::MoveAbsFocuser(uint32_t ticks)
//{
//    LOGF_DEBUG("MoveAbsPosition: %d", ticks);

//    if (isAbsolute == false)
//    {
//        LOG_ERROR("Focuser is not in Absolute mode. Please sync.");
//        return IPS_ALERT;
//    }

//    if (isParked != 0)
//    {
//        LOG_ERROR("Please unpark before issuing any motion commands.");
//        return IPS_ALERT;
//    }
//    if ( setPosition(ticks) )
//    {
//        FocusAbsPosNP.s = IPS_OK;
//        IDSetNumber(&FocusAbsPosNP, nullptr);
//        return IPS_OK;
//    }
//    return IPS_ALERT;
//}

//IPState GPIO_PWM_Focuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
//{
//    int32_t finalTicks = currentPosition + ((int32_t)ticks * (dir == FOCUS_INWARD ? -1 : 1));

//    LOGF_DEBUG("MoveRelPosition: %d", finalTicks);

//    if (isParked != 0)
//    {
//        LOG_ERROR("Please unpark before issuing any motion commands.");
//        return IPS_ALERT;
//    }

//    if ( setPosition(finalTicks) )
//    {
//        FocusRelPosNP.s = IPS_OK;
//        IDSetNumber(&FocusRelPosNP, nullptr);
//        return IPS_OK;
//    }
//    return IPS_ALERT;
//}


//void GPIO_PWM_Focuser::TimerHit()
//{

//    if ( ! isConnected() )
//        return;

//    int oldAbsStatus = FocusAbsPosNP.s;
//    int32_t oldPosition = currentPosition;

//    if ( getMaxPosition() )
//    {
//        if ( FocusMaxPosN[0].value != currentMaxPosition ) {
//            FocusMaxPosN[0].value = currentMaxPosition;
//            FocusMaxPosNP.s = IPS_OK;
//            IDSetNumber(&FocusMaxPosNP, nullptr);
//            SetFocuserMaxPosition(currentMaxPosition);
//        }
//    }
//    else
//        FocusMaxPosNP.s = IPS_ALERT;

//    if ( getStatus() )
//    {

//        StatusSP.s = IPS_OK;
//        if ( isMoving )
//        {
//            //LOG_INFO("Moving" );
//            FocusAbsPosNP.s = IPS_BUSY;
//            StatusS[1].s = ISS_ON;
//        }
//        else
//        {
//            if ( FocusAbsPosNP.s != IPS_IDLE )
//                FocusAbsPosNP.s = IPS_OK;
//            StatusS[1].s = ISS_OFF;
//        };

//        if ( isParked == 1 )
//        {
//            ParkSP.s = IPS_BUSY;
//            StatusS[2].s = ISS_ON;
//            ParkS[0].s = ISS_ON;
//        }
//        else if ( isParked == 2 )
//        {
//            ParkSP.s = IPS_OK;
//            StatusS[2].s = ISS_ON;
//            ParkS[0].s = ISS_ON;
//        }
//        else
//        {
//            StatusS[2].s = ISS_OFF;
//            ParkS[1].s = ISS_ON;
//            ParkSP.s = IPS_IDLE;
//        }

//        if ( isAbsolute )
//        {
//            StatusS[0].s = ISS_ON;
//            if ( FocusAbsPosN[0].min != 0 )
//            {
//                FocusAbsPosN[0].min = 0;
//                IDSetNumber(&FocusAbsPosNP, nullptr);
//            }
//        }
//        else
//        {
//            if ( FocusAbsPosN[0].min == 0 )
//            {
//                FocusAbsPosN[0].min = -FocusAbsPosN[0].max;
//                IDSetNumber(&FocusAbsPosNP, nullptr);
//            }
//            StatusS[0].s = ISS_OFF;
//        }

//    }
//    else
//        StatusSP.s = IPS_ALERT;

//    if ( getTemperature() )
//    {
//        WeatherNP.s = ( (WeatherN[0].value != currentTemperature) || (WeatherN[1].value != currentHumidity)) ? IPS_BUSY : IPS_OK;
//        WeatherN[0].value = currentTemperature;
//        WeatherN[1].value = currentHumidity;
//        WeatherN[2].value = pow(currentHumidity / 100, 1.0 / 8) * (112 + 0.9 * currentTemperature) + 0.1 * currentTemperature - 112;
//    }
//    else
//        WeatherNP.s = IPS_ALERT;

//    if ( FocusAbsPosNP.s != IPS_IDLE )
//    {
//        if ( getPosition() )
//        {
//            if ( oldPosition != currentPosition )
//            {
//                FocusAbsPosNP.s = IPS_BUSY;
//                StatusS[1].s = ISS_ON;
//                FocusAbsPosN[0].value = currentPosition;
//            }
//            else
//            {
//                StatusS[1].s = ISS_OFF;
//                FocusAbsPosNP.s = IPS_OK;
//            }
//            //if ( currentPosition < 0 )
//            // FocusAbsPosNP.s = IPS_ALERT;
//        }
//        else
//            FocusAbsPosNP.s = IPS_ALERT;
//    }


//    if ((oldAbsStatus != FocusAbsPosNP.s) || (oldPosition != currentPosition))
//        IDSetNumber(&FocusAbsPosNP, nullptr);

//    IDSetNumber(&WeatherNP, nullptr);
//    //IDSetSwitch(&SyncSP, nullptr);
//    IDSetSwitch(&StatusSP, nullptr);
//   IDSetSwitch(&ParkSP, NULL);

//    SetTimer(POLLMS);

//}


/****************************************************************
**
**
*****************************************************************/

//unsigned char GPIO_PWM_Focuser::calculate_checksum(gpioPWMCommand c)
//{
//    unsigned char z;

//    // calculate checksum
//    z = (c.M + c.k + c.a + c.b + c.c + c.d + c.addr) & 0xff;
//    return z;
//}

//bool GPIO_PWM_Focuser::send_command(char k, uint32_t l, unsigned char addr)
//{
//    gpioPWMCommand c;
//    int err_code = 0, nbytes_written = 0;
//    char gpioPWM_error[gpioPWM_ERROR_BUFFER];
//    unsigned char *x = (unsigned char *)&l;

//    switch(k)
//    {
//        case 'M':
//        case 'Z':
//            c.a = x[3];
//            c.b = x[2];
//            c.c = x[1];
//            c.d = x[0];
//            break;
//        case 'H':
//        case 'P':
//        case 'I':
//        case 'T':
//        case 'W':
//        case 'G':
//        case 'V':
//            c.a = 0;
//            c.b = 0;
//            c.c = 0;
//            c.d = 0;
//            break;
//        case 'R':
//            c.a = 0;
//            c.b = 0;
//            c.c = 0;
//            c.d = x[0];
//            break;
//        case 'A':
//        case 'C':
//            c.a = 0;
//            c.b = 0;
//            c.c = 0;
//            c.d = x[0];
//            break;
//        case 'B':
//        case 'D':
//            c.a = x[3];
//            c.b = x[2];
//            c.c = 0;
//            c.d = x[0];
//            break;
//        default:
//            DEBUGF(INDI::Logger::DBG_ERROR, "Unknown command: '%c'", k);
//            return false;
//    }
//    c.k = k;
//    c.addr = addr;
//    c.z = calculate_checksum(c);

//    LOGF_DEBUG("Sending command: c=%c, a=%hhu, b=%hhu, c=%hhu, d=%hhu ($%hhx), n=%hhu, z=%hhu", c.k, c.a, c.b, c.c, c.d, c.d, c.addr, c.z);

//    tcflush(PortFD, TCIOFLUSH);

//    if ( (err_code = tty_write(PortFD, (char *)&c, sizeof(c), &nbytes_written) != TTY_OK))
//    {
//        tty_error_msg(err_code, gpioPWM_error, gpioPWM_ERROR_BUFFER);
//        LOGF_ERROR("TTY error detected: %s", gpioPWM_error);
//        return false;
//    }

//    LOGF_DEBUG("Sending complete. Number of bytes written: %d", nbytes_written);

//    return true;
//}

//bool GPIO_PWM_Focuser::read_response()
//{
//    int err_code = 0, nbytes_read = 0, z;
//    char err_msg[gpioPWM_ERROR_BUFFER];

//    //LOG_DEBUG("Read response");

//    // Read a single response
//    if ( (err_code = tty_read(PortFD, (char *)&currentResponse, sizeof(currentResponse), 5, &nbytes_read)) != TTY_OK)
//    {
//        tty_error_msg(err_code, err_msg, 32);
//        LOGF_ERROR("TTY error detected: %s", err_msg);
//        return false;
//    }
//    LOGF_DEBUG("Response: %c, a=%hhu, b=%hhu, c=%hhu, d=%hhu ($%hhx), n=%hhu, z=%hhu", currentResponse.k, currentResponse.a, currentResponse.b, currentResponse.c, currentResponse.d, currentResponse.d, currentResponse.addr, currentResponse.z);

//    if ( nbytes_read != sizeof(currentResponse) )
//    {
//        LOGF_ERROR("Number of bytes read: %d, expected: %d", nbytes_read, sizeof(currentResponse));
//        return false;
//    }

//    z = calculate_checksum(currentResponse);
//    if ( z != currentResponse.z )
//    {
//        LOGF_ERROR("Response checksum in not correct %hhu, expected: %hhu", currentResponse.z, z );
//        return false;
//    }

//    if ( currentResponse.k == '!' )
//    {
//        LOG_ERROR("Focuser reported unrecognized command.");
//        return false;
//    }

//    if ( currentResponse.k == '?' )
//    {
//        LOG_ERROR("Focuser reported bad checksum.");
//        return false;
//    }

//    return true;
//}

//bool GPIO_PWM_Focuser::dispatch_command(char k, uint32_t l, unsigned char addr)
//{
//    LOG_DEBUG("send_command");
//    if ( send_command(k, l, addr) )
//    {
//        if ( read_response() )
//        {
//            LOG_DEBUG("check currentResponse.k");
//            if ( currentResponse.k == k )
//                return true;
//        }
//    }
//    return false;
//}

/****************************************************************
**
**
*****************************************************************/

const char * GPIO_PWM_Focuser::getDefaultName()
{
    return (char *)"PWM Focuser via GPIO pins";
}
