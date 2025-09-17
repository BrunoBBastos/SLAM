#include <stdio.h>
#include "driver/gpio.h" 
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "math.h"
#include "nvs_flash.h"
#include "wifi.h"
#include "MQTT_lib.h"

#include "PID_control.h"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  CONSTS

// Cofiguração do pwm
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Resolução do duty cycle
#define LEDC_FREQUENCY          (20000) 
#define LEDC_CHANNEL_MESQA      LEDC_CHANNEL_0
#define LEDC_CHANNEL_MESQB      LEDC_CHANNEL_1
#define LEDC_CHANNEL_MDIRA      LEDC_CHANNEL_2
#define LEDC_CHANNEL_MDIRB      LEDC_CHANNEL_3
#define ESP_INTR_FLAG_DEFAULT   0

// Gpio usado
#define MOTOR_ESQ_A             41  //4
#define MOTOR_ESQ_B             42 //18
#define ENCODER_ESQ_A           1 //16
#define ENCODER_ESQ_B           40 //17
#define MOTOR_DIR_A             14 //25
#define MOTOR_DIR_B             47 //26
#define ENCODER_DIR_A           18 //32
#define ENCODER_DIR_B           8  //33

// Parâmetros de uso dos encoders
#define BORDAS_POR_VOLTA        1496
#define DELTA_T                 25

// Parâmetros do robô
#define RADIUS_WHEEL            0.0335 // raio das rodas em metros
#define WHEEL_BASE              0.1113    // distância entre rodas

// Parâmetros do controlador de velocidade dos motores
// MELHORIA: normalizar saída do controle para duty (-1% ... 1%)
#define PID_KP                  75//55*8 //5.0 // 25
#define PID_KI                  300//300*8 //30.00 // 30
#define PID_KD                  0.0
#define PID_TAU                 0.05
#define PID_LIM_MAX             (1023)
#define PID_LIM_MIN             (-1023)

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  GLOBAIS

static const char *TAG = "ROBOT";

enum Mode
{
    RESET,
    CONTROL,
    STANDBY
};

int operation_mode = RESET;
volatile int sinais_encoder_esq = 0, sinais_encoder_dir = 0;
volatile float angular_speed_wheels[2] = {0.0f, 0.0f};
volatile float pose[3] = {0.0f, 0.0f, 0.0f};

float left_speed_setpoint = 0.0f;
float right_speed_setpoint = 0.0f;

bool new_measurement = false;

PIDController pid_left_motor = {PID_KP, PID_KI, PID_KD,
                                PID_TAU, PID_LIM_MIN, PID_LIM_MAX,
                                DELTA_T/1000.0f};
PIDController pid_right_motor = {PID_KP, PID_KI, PID_KD,
                                PID_TAU, PID_LIM_MIN, PID_LIM_MAX,
                                DELTA_T/1000.0f};

gptimer_handle_t timer;
gptimer_handle_t timerEncoder;
gptimer_handle_t timerMQTT;

SemaphoreHandle_t xBinarySemaphoreMQTTPublish = NULL;
QueueHandle_t xQueueEncoderTimer = NULL;
QueueHandle_t xQueueAngularSpeed = NULL;
QueueHandle_t xQueueOdometry = NULL;
QueueHandle_t xQueueMQTT = NULL;
SemaphoreHandle_t xMutexAngularSpeed = NULL;

TaskHandle_t xTaskHandleUpdateControl;// = NULL;
TaskHandle_t xTaskHandleOdometry;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  SETUP
static void encoder_pin_init(void)
{
    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL<<ENCODER_ESQ_A)|
                        (1ULL<<ENCODER_ESQ_B)|
                        (1ULL<<ENCODER_DIR_A)|
                        (1ULL<<ENCODER_DIR_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_config);
}


// Configuração dos pinos de controle dos motores
static void ledc_init(void)
{
    // Configura LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Array de configs de canal
    ledc_channel_config_t ledc_channels[] = {
        {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_MESQA,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = MOTOR_ESQ_A,
            .duty           = 0,
            .hpoint         = 0
        },
        {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_MESQB,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = MOTOR_ESQ_B,
            .duty           = 0,
            .hpoint         = 0
        },
        {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_MDIRA,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = MOTOR_DIR_A,
            .duty           = 0,
            .hpoint         = 0
        },
        {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_MDIRB,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = MOTOR_DIR_B,
            .duty           = 0,
            .hpoint         = 0
        }
    };

    // Aplicar as configs de canal
    for (size_t i = 0; i < sizeof(ledc_channels)/sizeof(ledc_channels[0]); ++i) {
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channels[i]));
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  UTILS

float angleWrap(float ang)
{
    if (ang > M_PI)
    {
        return ang - 2 * M_PI;
    }
    else if (ang < -M_PI)
    {
        return ang + 2 * M_PI;
    }
    return ang;
}

// Função para setar o pwm dos motores
void set_motor(int pwm, int channelA, int channelB)
{
    if(pwm >= 0)
    {
        ledc_set_duty(LEDC_MODE, channelA, pwm);
        ledc_set_duty(LEDC_MODE, channelB, 0);
    }
    else
    {
        ledc_set_duty(LEDC_MODE, channelA, 0);
        ledc_set_duty(LEDC_MODE, channelB, abs(pwm));
    }

    ledc_update_duty(LEDC_MODE, channelA);
    ledc_update_duty(LEDC_MODE, channelB);
}

// Função para acionar os dois motores
void drive_motors(int left_pwm, int right_pwm){
    set_motor(left_pwm, LEDC_CHANNEL_MESQA, LEDC_CHANNEL_MESQB);
    set_motor(right_pwm, LEDC_CHANNEL_MDIRA, LEDC_CHANNEL_MDIRB);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  INTERRUPTS

static void IRAM_ATTR left_encoder_intr_handler(void *args)
{
    bool A = gpio_get_level(ENCODER_ESQ_A);
    bool B = gpio_get_level(ENCODER_ESQ_B);
    int pinNumber = (int)args;
    if(pinNumber == ENCODER_ESQ_A)
    {
        if(A == B) sinais_encoder_esq--;
        else sinais_encoder_esq++;
    }
    else
    {
        if(A == B) sinais_encoder_esq++;
        else sinais_encoder_esq--;
    }
}

static void IRAM_ATTR right_encoder_intr_handler(void *args)
{
    bool A = gpio_get_level(ENCODER_DIR_A);
    bool B = gpio_get_level(ENCODER_DIR_B);
    int pinNumber = (int)args;
    if(pinNumber == ENCODER_DIR_A)
    {
        if(A == B) sinais_encoder_dir--;
        else sinais_encoder_dir++;
    }
    else
    {
        if(A == B) sinais_encoder_dir++;
        else sinais_encoder_dir--;
    }
}


static void IRAM_ATTR timer_isr_encoders(void *arg)
{
    int pulses[2] = {sinais_encoder_esq, sinais_encoder_dir};
    xQueueSendFromISR(xQueueEncoderTimer, pulses, NULL);
    sinais_encoder_esq = 0;
    sinais_encoder_dir = 0;
}

// Timer do MQTT
static void IRAM_ATTR timer_isr_MQTT(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    xSemaphoreGiveFromISR(xBinarySemaphoreMQTTPublish, xHigherPriorityTaskWoken);
}

void init_timers()
{
    // Configuração de Timer dos Encoders
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };
    gptimer_new_timer(&timer_config, &timerEncoder);

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0, // counter will reload with 0 on alarm event
        .alarm_count = DELTA_T * 1000, // period = 25ms @resolution 1MHz
        .flags.auto_reload_on_alarm = true, // enable auto-reload
    };
    gptimer_set_alarm_action(timerEncoder, &alarm_config);

    gptimer_event_callbacks_t isr = {
        .on_alarm = timer_isr_encoders, // register user callback
    };

    gptimer_register_event_callbacks(timerEncoder, &isr, NULL);
    gptimer_enable(timerEncoder);
    gptimer_start(timerEncoder);

    // Configuração de Timer do MQTT
    gptimer_config_t timer_config_MQTT = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };
    gptimer_new_timer(&timer_config_MQTT, &timerMQTT);

    gptimer_alarm_config_t alarm_MQTT_config = {
        .reload_count = 0, // counter will reload with 0 on alarm event
        // .alarm_count = DELTA_T * 10 * 1000, // period = 25ms @resolution 1MHz
        .alarm_count = DELTA_T * 1000, // period = 25ms @resolution 1MHz
        .flags.auto_reload_on_alarm = true, // enable auto-reload
    };
    gptimer_set_alarm_action(timerMQTT, &alarm_MQTT_config);

     gptimer_event_callbacks_t isr_MQTT = {
        .on_alarm = timer_isr_MQTT, // register user callback
    };

    gptimer_register_event_callbacks(timerMQTT, &isr_MQTT, NULL);
    gptimer_enable(timerMQTT);
    gptimer_start(timerMQTT);
    
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  MQTT

void vTaskPublishMQTT(void)
{
    float pose[3] = {0.0f, 0.0f, 0.0f};
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;

    while(1)
   {
        if(xSemaphoreTakeFromISR(xBinarySemaphoreMQTTPublish, xHigherPriorityTaskWoken)){
            if(xQueueReceive(xQueueMQTT, pose, portMAX_DELAY) == pdTRUE)
            {
                static char x[20];
                sprintf(x, "%f", pose[0]);
                if(mqtt_connected())
                {
                    mqtt_publish("SLAM/robot2/odometria/x", x, 0, 0);
                }

                if(xSemaphoreTake(xMutexAngularSpeed, portMAX_DELAY== pdTRUE)){
                    static char str[20];
                    sprintf(str, "%f", angular_speed_wheels[0]);
                    xSemaphoreGive(xMutexAngularSpeed);
                    if(mqtt_connected())
                    {
                        mqtt_publish("SLAM/robot2/vel/left", str, 0, 0);
                    }
                }
            }
        }
        vTaskDelay(1);
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  ROBOT ROUTINE

void vTaskGetOdometry()
{
    const float dt_s = DELTA_T/1000.0f;

    while(1)
    {
        if(xSemaphoreTake(xMutexAngularSpeed, portMAX_DELAY) == pdTRUE) 
        {
            float speed_left_wheel = angular_speed_wheels[0] * RADIUS_WHEEL;
            float speed_right_wheel = angular_speed_wheels[1] * RADIUS_WHEEL;
            // xSemaphoreGive(xMutexAngularSpeed);
            
            if(speed_left_wheel == speed_right_wheel)
            {
                pose[0] += speed_left_wheel * dt_s * cos(pose[2]);
                pose[1] += speed_right_wheel * dt_s * sin(pose[2]);
            }
            else
            {
                float robot_angular_speed = (speed_right_wheel -
                                             speed_left_wheel) / WHEEL_BASE;
                float curvature_radius = (WHEEL_BASE / 2) * 
                                        (speed_left_wheel + speed_right_wheel) /
                                        (speed_right_wheel - speed_left_wheel);
                float curvature_center_x = pose[0] +
                                           curvature_radius * sin(pose[2]);
                float curvature_center_y = pose[1] +
                                           curvature_radius * cos(pose[2]);
                float arc = dt_s * robot_angular_speed;

                pose[0] = cos(arc) * (pose[0] - curvature_center_x) -
                          sin(arc) * (pose[1] - curvature_center_y) +
                          curvature_center_x;
                pose[1] = cos(arc) * (pose[1] - curvature_center_y) -
                          sin(arc) * (pose[0] - curvature_center_x) +
                          curvature_center_y;
                pose[2] = angleWrap(pose[2] + arc);
            }

            xQueueSend(xQueueMQTT, pose, portMAX_DELAY);
            xSemaphoreGive(xMutexAngularSpeed);
            vTaskSuspend(NULL);
        }
        vTaskDelay(1);
    }
}

void vTaskGetAngularSpeed(void)
{
    int pulses[2] = {0, 0};
    const float delta_phi = 2*M_PI / BORDAS_POR_VOLTA;
    const float dt_s = DELTA_T / 1000.0f;
    const float arc_step_per_dt = delta_phi / dt_s;
    while(1)
    {
        if(xSemaphoreTake(xMutexAngularSpeed, portMAX_DELAY== pdTRUE)){
            if(xQueueReceive(xQueueEncoderTimer, pulses, portMAX_DELAY) == pdTRUE)
            {
                angular_speed_wheels[0] = pulses[0] * arc_step_per_dt;
                angular_speed_wheels[1] = pulses[1] * arc_step_per_dt;

                vTaskResume(xTaskHandleOdometry);
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xSemaphoreGive(xMutexAngularSpeed);
                vTaskNotifyGiveFromISR(xTaskHandleUpdateControl, &xHigherPriorityTaskWoken);
            } 
        }
        vTaskDelay(1);
    }
}

void vTaskUpdateControl(void)
{
    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        {
            if(xSemaphoreTake(xMutexAngularSpeed, portMAX_DELAY== pdTRUE))
            {
                PIDController_Update(&pid_left_motor,
                                     left_speed_setpoint,
                                     angular_speed_wheels[0]);
                PIDController_Update(&pid_right_motor,
                                     right_speed_setpoint,
                                     angular_speed_wheels[1]);

                drive_motors(pid_left_motor.output,
                            pid_right_motor.output);

                xSemaphoreGive(xMutexAngularSpeed);
            }
        }
        vTaskDelay(1);
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  MAIN

void app_main(void)
{
     // Inicializando NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();    //Inicializando o WiFi. Função da Lib criada, wifi.h
    ESP_LOGI(TAG, "WiFi foi inicializado!");

    mqtt_start();       //Iniciando conexão MQTT. Função da Lib criada, MQTT.h
    ESP_LOGI(TAG, "MQTT foi inicializado!");

    xQueueEncoderTimer = xQueueCreate(50, sizeof(int) * 2);
    xBinarySemaphoreMQTTPublish = xSemaphoreCreateBinary();
    xQueueAngularSpeed = xQueueCreate(50, sizeof(float) * 2);
    xQueueMQTT = xQueueCreate(50, sizeof(float) * 3);
    xMutexAngularSpeed = xSemaphoreCreateMutex();

    ledc_init();
    encoder_pin_init();
    init_timers();

    PIDController_Init(&pid_left_motor);
    PIDController_Init(&pid_right_motor);
    
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(ENCODER_ESQ_A, left_encoder_intr_handler,
                        (void *)ENCODER_ESQ_A);
    gpio_isr_handler_add(ENCODER_ESQ_B, left_encoder_intr_handler,
                        (void *)ENCODER_ESQ_B);
    gpio_isr_handler_add(ENCODER_DIR_A, right_encoder_intr_handler,
                        (void *)ENCODER_DIR_A);
    gpio_isr_handler_add(ENCODER_DIR_B, right_encoder_intr_handler,
                        (void *)ENCODER_DIR_B);

    xTaskCreate(vTaskGetAngularSpeed,
                "vTaskGetAngularSpeed", 
                2*configMINIMAL_STACK_SIZE, 
                NULL, 
                4, 
                NULL);

     xTaskCreate(vTaskGetOdometry,
                "vTaskGetOdometry",
                2*configMINIMAL_STACK_SIZE,
                NULL,
                3,
                &xTaskHandleOdometry);

    xTaskCreate(vTaskPublishMQTT,
                "vTaskPublishMQTT",
                2*configMINIMAL_STACK_SIZE,
                NULL,
                2,
                NULL);

    xTaskCreate(vTaskUpdateControl,
                 "vTaskUpdateControl",
                 2*configMINIMAL_STACK_SIZE,
                 NULL,
                 3,
                 &xTaskHandleUpdateControl);

    while(true)
    {
        // ESP_LOGI(TAG, "sinal de pwm: %f", pid_left_motor.output);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void change_operation_mode(int mode)
{
    switch(mode)
    {
        case RESET:
        {
            operation_mode = RESET;
            pose[0] = 0.0f;
            pose[1] = 0.0f;
            pose[2] = 0.0f;
            left_speed_setpoint = 0.0f;
            right_speed_setpoint = 0.0f;
            PIDController_Init(&pid_left_motor);
            PIDController_Init(&pid_right_motor);
            break;
        }
        
        case CONTROL:
        {
            operation_mode = CONTROL;
            PIDController_Init(&pid_left_motor);
            PIDController_Init(&pid_right_motor);
            break;
        }

        case STANDBY:
        {
            operation_mode = STANDBY;
            PIDController_Init(&pid_left_motor);
            PIDController_Init(&pid_right_motor);
            break;
        }
    }
}