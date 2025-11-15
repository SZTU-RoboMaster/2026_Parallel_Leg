#include "usb_device.h"
#include "cmsis_os.h"
#include "RoboMaster_Protocol.h"
#include "usb_task.h"

/***************************************************************************************************
 *                                          变量定义                                                *
 ***************************************************************************************************/
static uint8_t usb_buf[128];
extern QueueHandle_t CDC_send_queue;


rc_info_t rc_data;

void usb_task(void const*pvParameters)
{
    MX_USB_DEVICE_Init();
    while(1)
    {
        if(xQueueReceive( CDC_send_queue, usb_buf, 10 ) == pdTRUE)
        {
            rm_dequeue_send_data(usb_buf,128);
        }
        osDelay(2);
    }

}