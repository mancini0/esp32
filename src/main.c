#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


QueueHandle_t xQueue;

void send(void* pvParameters){
    const TickType_t blockTime = pdMS_TO_TICKS( 200 );
    int32_t lValueToSend = 3;
    
  for(;;){
      vTaskDelay(blockTime);
      xQueueSend(xQueue, &lValueToSend, pdMS_TO_TICKS(100));
  }
}

void receive(void* pvParameters){
    const TickType_t blockTime = pdMS_TO_TICKS(25);
    int32_t lReceivedValue;
    BaseType_t status;
  for(;;){
      vTaskDelay(blockTime);
      status = xQueueReceive(xQueue, &lReceivedValue, blockTime);
      if(status==pdPASS){
          printf("got_it\n");
      }
      else{
          printf("nothing_received\n");
      }
      fflush(stdout);
  }
}

void app_main() {
    xQueue= xQueueCreate( 5, sizeof( int32_t));
    xTaskCreate(send,"send task",2048,NULL,1,NULL);
    xTaskCreate(receive,"rcv task",2048,NULL,1,NULL);
}
