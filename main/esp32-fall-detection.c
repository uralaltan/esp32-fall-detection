#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void imu_task(void *arg) {
    for (;;) {
        imu_read_and_print();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    imu_init();
    xTaskCreate(imu_task, "imu", 2048, NULL, 5, NULL);
}
