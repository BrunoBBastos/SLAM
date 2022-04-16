/**************************************************************
   Criada por Bruno Bezerra Bastos com o intuito de servir ao projeto de estudos sobre SLAM.
   Escrita em 15/04/2022.
 **************************************************************/

#include "Arduino.h"
#include "MsTimer2.h"

class RoboUniciclo
{
    public:
        // RoboUniciclo();

        void RoboUniciclo::iniciar()
        {
            // MsTimer2::set(_dt * 1000, odometria);
            // MsTimer2::start();
        }

        // GETTERS
        // float RoboUniciclo::getPose(float pose[3])
        // {
        //     pose[0] = getPoseX();
        //     pose[1] = getPoseY();
        //     pose[2] = getPoseTheta();
        // }
        // float RoboUniciclo::getPoseX()
        // {
        //     return _pose[0];
        // }
        // float RoboUniciclo::getPoseY()
        // {
        //     return _pose[1];
        // }
        // float RoboUniciclo::getPoseTheta()
        // {
        //     return _pose[2];
        // }

        // int RoboUniciclo::convertePWM(float sinal)
        // {
        // int pwm = sinal * 255L / maxVel;
        // int sig = abs(pwm)/ pwm;
        // pwm = constrain(abs(pwm), 50, 255) * sig;
        // return pwm;
        // }

    // private:
        float raio = 31.75f / 1000; // Raio das rodas
        float l = 120.0f / 1000; // Distância entre os pontos das rodas que tocam o chão
        float rpm = 100.0f; // Rotações por minuto do eixo do motor
        float ppr = 2091.0f; // Pulsos do encoder por rotação
        float dPhi = ppr / (2 * PI); // Rotação por pulso contado
        float pose[3] = {0.0f, 0.0f, 0.0f}; // Pose do robô
        float dt = 20.0f / 1000; // Intervalo de tempo para coleta de pulsos dos encoders em [s]
        volatile int pulsosR = 0, pulsosL = 0; // Pulsos coletados dos encoders direito e esquerdo

        float maxVel = (rpm/60) * 2 * PI * raio;

        
};

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
  else
  {
    return ang;
  }
}

float medirDistancia(float p1[2], float p2[2])
{
  return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));
}

