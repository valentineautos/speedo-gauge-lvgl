#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "TCA9554PWR.h"
#include "PCF85063.h"
#include "QMI8658.h"
#include "ST7701S.h"
#include "CST820.h"
#include "SD_SPI.h"
#include "LVGL_Driver.h"
#include "LVGL_Example.h"
#include "Speedo_demo.h"
#include "Wireless.h"

void app_main(void)
{   
    Wireless_Init();
    Flash_Searching();
    I2C_Init();
    PCF85063_Init();
    QMI8658_Init();
/********************* EXIO *********************/
    EXIO_Init();                    // Example Initialize EXIO

    LCD_Init();
    Touch_Init();
    SD_Init();
    LVGL_Init();
/********************* Demo *********************/

    Build_UI();


    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
        PCF85063_Read_Time(&datetime);
        getAccelerometer();
    }
}
