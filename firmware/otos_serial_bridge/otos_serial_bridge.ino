/*
    SPDX-License-Identifier: MIT

    OTOS serial bridge firmware.
    Streams: x,y,h,vx,vy,wz,ax,ay,az,sx,sy,sh,svx,svy,swz,sax,say,saz
*/

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

QwiicOTOS myOtos;

static const unsigned long kLoopPeriodMs = 20;  // 50 Hz
static const float kScalarMin = 0.872f;
static const float kScalarMax = 1.127f;

unsigned long lastPublishMs = 0;
char commandBuffer[64];
uint8_t commandIndex = 0;

bool isScalarInRange(float value)
{
    return value >= kScalarMin && value <= kScalarMax;
}

void emitCsv(
    const sfe_otos_pose2d_t &pos,
    const sfe_otos_pose2d_t &vel,
    const sfe_otos_pose2d_t &acc,
    const sfe_otos_pose2d_t &posStdDev,
    const sfe_otos_pose2d_t &velStdDev,
    const sfe_otos_pose2d_t &accStdDev)
{
    Serial.print(pos.x, 4);
    Serial.print(',');
    Serial.print(pos.y, 4);
    Serial.print(',');
    Serial.print(pos.h, 4);
    Serial.print(',');

    Serial.print(vel.x, 4);
    Serial.print(',');
    Serial.print(vel.y, 4);
    Serial.print(',');
    Serial.print(vel.h, 4);
    Serial.print(',');

    Serial.print(acc.x, 4);
    Serial.print(',');
    Serial.print(acc.y, 4);
    Serial.print(',');
    Serial.print(acc.h, 4);
    Serial.print(',');

    Serial.print(posStdDev.x, 4);
    Serial.print(',');
    Serial.print(posStdDev.y, 4);
    Serial.print(',');
    Serial.print(posStdDev.h, 4);
    Serial.print(',');

    Serial.print(velStdDev.x, 4);
    Serial.print(',');
    Serial.print(velStdDev.y, 4);
    Serial.print(',');
    Serial.print(velStdDev.h, 4);
    Serial.print(',');

    Serial.print(accStdDev.x, 4);
    Serial.print(',');
    Serial.print(accStdDev.y, 4);
    Serial.print(',');
    Serial.println(accStdDev.h, 4);
}

void processCommand(char *cmd)
{
    if (strcmp(cmd, "R") == 0)
    {
        myOtos.resetTracking();
        Serial.println("#OK,R");
        delay(500);
        return;
    }

    if (strcmp(cmd, "C") == 0)
    {
        myOtos.calibrateImu();
        myOtos.resetTracking();
        Serial.println("#OK,C");
        return;
    }

    if (strncmp(cmd, "S,", 2) == 0)
    {
        float linearScalar = 0.0f;
        float angularScalar = 0.0f;
        int parsed = sscanf(cmd + 2, "%f,%f", &linearScalar, &angularScalar);
        if (parsed == 2 && isScalarInRange(linearScalar) && isScalarInRange(angularScalar))
        {
            myOtos.setLinearScalar(linearScalar);
            myOtos.setAngularScalar(angularScalar);
            myOtos.resetTracking();
            Serial.println("#OK,S");
            return;
        }

        Serial.println("#ERR,S");
        return;
    }

    Serial.println("#ERR,UNKNOWN");
}

void handleSerialInput()
{
    while (Serial.available() > 0)
    {
        char incoming = static_cast<char>(Serial.read());
        if (incoming == '\r')
        {
            continue;
        }

        if (incoming == '\n')
        {
            commandBuffer[commandIndex] = '\0';
            if (commandIndex > 0)
            {
                processCommand(commandBuffer);
            }
            commandIndex = 0;
            continue;
        }

        if (commandIndex < sizeof(commandBuffer) - 1)
        {
            commandBuffer[commandIndex++] = incoming;
        }
        else
        {
            commandIndex = 0;
            Serial.println("#ERR,OVERFLOW");
        }
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000)
    {
    }

    Wire1.begin(); // Change to Wire if using the default I2C pins

    while (!myOtos.begin(Wire1))
    {
        Serial.println("#WAIT,OTOS_NOT_CONNECTED");
        delay(1000);
    }

    myOtos.calibrateImu();
    myOtos.setLinearUnit(kSfeOtosLinearUnitMeters);
    myOtos.setAngularUnit(kSfeOtosAngularUnitRadians);
    myOtos.setLinearScalar(1.0f);
    myOtos.setAngularScalar(1.0f);
    sfe_otos_pose2d_t offset = {0.1017, 0, 0}; //TODO Don't use hardcoded value
    myOtos.setOffset(offset);

    myOtos.resetTracking();

    Serial.println("#READY");
}

void loop()
{
    handleSerialInput();

    unsigned long now = millis();
    if (now - lastPublishMs < kLoopPeriodMs)
    {
        return;
    }
    lastPublishMs = now;

    sfe_otos_pose2d_t pos;
    sfe_otos_pose2d_t vel;
    sfe_otos_pose2d_t acc;
    sfe_otos_pose2d_t posStdDev;
    sfe_otos_pose2d_t velStdDev;
    sfe_otos_pose2d_t accStdDev;

    myOtos.getPosVelAccAndStdDev(pos, vel, acc, posStdDev, velStdDev, accStdDev);
    emitCsv(pos, vel, acc, posStdDev, velStdDev, accStdDev);
}
