#include <Arduino.h>
#include <FastLED.h>
#include <algorithm>
#include "flight_control.hpp"
#include "pid.hpp"

#define SEC_TO_ETAPA(s) s*100

// Definir variables globales
extern float Altitude2; // Inicializa la altitud
extern float Voltage;  // Ejemplo de voltaje (aquí lo asignas según el sensor)
extern Filter Thrust_filtered;  // Filtro de Thrust

// Tarea 1: Despegue hasta 15 cm
TaskHandle_t TaskManagerHandle;
TaskHandle_t Taskloop400Handle;
TaskHandle_t TaskTakeoffHandle;
TaskHandle_t TaskStabilizeHandle;
TaskHandle_t TaskLandingHandle;
TaskHandle_t TaskSTOPHandle;

float min (volatile float& a, float b) {
    return (a < b) ? a : b;
}

float max (volatile float& a, float b) {
    return (a > b) ? a : b;
}
int etapa = 0;

void TaskTakeoff(void *pvParameters) {


    while(1) {
        if(0){
            Thrust_command += 0.1f; // Aumentar el Thrust de forma gradual
            Thrust_command = min(Thrust_command, get_trim_duty(Voltage));  // Limitar el Thrust máximo según el voltaje
            Thrust_command = Thrust_filtered.update(Thrust_command, Interval_time);

            // Espera hasta el siguiente ciclo de control
            ++etapa;
            vTaskDelay(pdMS_TO_TICKS(10));  // 100 ms de espera entre cada ciclo
        }
        else {
            etapa = 0;
            // Al alcanzar la altura, pasar a estabilizar
            xTaskNotifyGive(TaskStabilizeHandle);
            vTaskDelete(NULL);  // Eliminar la tarea después de despegar
        }
    }
}

// Tarea 2: Estabilización durante 10 segundos
void TaskStabilize(void *pvParameters) {
    // Esperar la notificación de que el despegue ha terminado
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Mode = FLIGHT_MODE;
    Thrust_command = Thrust_filtered.update(Thrust_command, Interval_time);
    // Mantener el dron estable durante 10 segundos
    vTaskDelay(pdMS_TO_TICKS(1000));  // 10 segundos de estabilización
    // Cuando termine el tiempo de estabilización, pasar a aterrizar
    xTaskNotifyGive(TaskLandingHandle);
    vTaskDelete(NULL);  // Eliminar la tarea después de estabilizar
}

// Tarea 3: Aterrizaje suave
void TaskLanding(void *pvParameters) {
    // Esperar la notificación de que la estabilización ha terminado
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (1)
    {
        Mode = AUTO_LANDING_MODE;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Apagar los motores una vez que haya aterrizado
    xTaskNotifyGive(TaskSTOPHandle);
    vTaskDelete(NULL);  // Eliminar la tarea después de aterrizar
}

// Tarea 3: Aterrizaje suave
void TaskSTOP(void *pvParameters) {
    // Esperar la notificación de que la estabilización ha terminado
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Mode = PARKING_MODE;
    // Apagar los motores una vez que haya aterrizado
    motor_stop();
    vTaskDelete(NULL);  // Eliminar la tarea después de aterrizar
}

void TaskManager(void *pvParameters) {

    while (1)
    {

        if(etapa <= SEC_TO_ETAPA(1)) {
             Mode = FLIGHT_MODE;
             Thrust_command += 0.5f; // Aumentar el Thrust de forma gradual
             Thrust_command = min(Thrust_command, get_trim_duty(Voltage));  // Limitar el Thrust máximo según el voltaje
             Thrust_command = Thrust_filtered.update(Thrust_command, Interval_time);
             USBSerial.println("Rise");
            USBSerial.println(Thrust_command);

         }
         else if(etapa <= SEC_TO_ETAPA(9)){
             Mode = FLIGHT_MODE;
             Mode = FLIGHT_MODE;
             Thrust_command = Thrust_filtered.update(Thrust_command, Interval_time);
            USBSerial.println("Stabilize");
            USBSerial.println(Thrust_command);
        }
         else if(etapa <= SEC_TO_ETAPA(15)) {
             Mode = AUTO_LANDING_MODE;
            USBSerial.println("Stope");
            USBSerial.println(Thrust_command);
         }
         else {
            Mode = PARKING_MODE;
            motor_stop();
        }
        
        ++etapa;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
}

void Taskloop400(void *pvParameters) {
    while (1)
    {
        //loop_400Hz();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
}

void setup() {
    init_copter();
    delay(100);

    // Crear las tareas
    xTaskCreatePinnedToCore(TaskManager, "TaskManager",configMINIMAL_STACK_SIZE + 4096,  NULL, 1, &TaskManagerHandle,1);
    //xTaskCreate(Taskloop400, "Taskloop400", configMINIMAL_STACK_SIZE + 4096, NULL, 1, &Taskloop400Handle);


}

void loop() {
    // El bucle principal puede estar vacío o se puede agregar código adicional si es necesario
    // Aquí no es necesario hacer nada, ya que las tareas de FreeRTOS manejan todo el control
    loop_400Hz();
}
