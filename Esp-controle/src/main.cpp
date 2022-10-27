#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

int tDelta = 20;
float dt = tDelta/1000.0;

float kpLinear = 0.05, kpAngular = 0.2;

float Pose[3] = {0.0, 0.0, 0.0};
float Referencia[3] = {0.0, 0.0, 0.0};

WiFiServer server(80);
WiFiClient client;

// #####################################################################################
// #####################################################################################
// #####################################################################################
// #####################################################################################

bool seguirRef()
{
    float pos[2] = {Pose[0], Pose[1]};
    float ref[2] = {Referencia[0], Referencia[1]};
    float dist = distancia2D(pos, ref); 
    if(dist > 0.05)
    {
        float ctrl[2];
        gerarSinalControlePosicao(Pose, Referencia, ctrl);
        Serial.print('V ');
        Serial.print(ctrl[0]);
        Serial.print(' ');
        Serial.println(ctrl[1]);
        return 1;
    }
    else
    {
        Serial.print('V ');
        Serial.print(0.0);
        Serial.print(' ');
        Serial.println(0.0);
    }
    return 0;
}

void gerarSinalControlePosicao(float Pose[3], float referencia[3], float ctrl[2])
{
    float dY = (referencia[1] - Pose[1]);
    float dX = (referencia[0] - Pose[0]);
    float THETAref = atan2(dY, dX);
    THETAref = angleWrap(THETAref);

    float erroAngular = angleWrap(THETAref - Pose[2]);
    float erroLinear = sqrt(pow(dX, 2) + pow(dY, 2)) * cos(erroAngular);

    float v = kpLinear * erroLinear;
    float w = kpAngular * erroAngular;
    ctrl[0] = v - w;
    ctrl[1] = v + w;
}

float angleWrap(float ang)
{
    if (ang > PI)
    {
    return ang - 2 * PI;
    }
    else if (ang < -PI)
    {
    return ang + 2 * PI;
    }
    return ang;
}

double distancia2D(float p1[2], float p2[2])
{
    return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2));   
}

void ouvirSerial()
{
    int cmd;
    if (Serial.available() > 0)
    {
        cmd = Serial.read();
        switch(cmd)
        {
            // Informar o último incremento de posição
            case 'O':
            {
            float x = Serial.parseFloat(SKIP_WHITESPACE);
            float y = Serial.parseFloat(SKIP_WHITESPACE);
            float t = Serial.parseFloat(SKIP_WHITESPACE);
            
            Pose[0] += x;
            Pose[1] += y;
            Pose[2] += t;
            }
            break;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    WiFiManager wifimanager;
    wifimanager.autoConnect();
    server.begin();
}

void loop()
{

    ouvirSerial();
}