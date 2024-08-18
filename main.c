/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

Задание
Реализовать многозадачную систему, в которой сообщения между задачами передаются через общую
очередь. Основные условия:
1. Система создаётся с тремя задачами и одной очередью.
2. Очередь имеет размер 100. Элементом очереди является структура, пример:
  typedef enum {
    MSG_ID_KEYBOARD = 0,
    MSG_ID_ADC
  } msgID;
  typedef struct {
    msgID id;
    uint16_t msgVal;
  } queueMsg;
3. Задача 1 выполняет опрос пользовательской кнопки на отладочной плате. При нажатии формируется
сообщение в очередь с идентификатором события нажатия кнопки MSG_ID_KEYBOARD.
4. Задача 2 выполняет запуск АЦП и преобразование по выбранному каналу с периодом 500мс. Результат
преобразования упаковывает в сообщение для очереди с идентификатором события АЦП
MSG_ID_ADC и численным значением преобразования.
5. Задача 3 выполняет чтение из очереди (таймаут чтения 1000 мс). При получении сообщения, для
каждого идентификатора выполняется соответствующее действие:
  для события MSG_ID_KEYBOARD – переключается состояние пользовательского
светодиода на противоположное (исходное состояние для светодиода после загрузки
системы – выключен);
  для события MSG_ID_ADC – обновляется значение локальной переменной, содержащей
значение предыдущего преобразования и происходит вывод этого значения через терминал
на ПК в виде форматной строки «ADC = %d\n».
6. Выполнить сборку, запуск и тестирование проекта на отладочной плате.
Критерии готовности (Definition of Done)
1. Проект собирается и загружается в отладочную плату без ошибок и предупреждений.
2. Начальные условия при включении системы соблюдаются.
3. По последовательному порту передаётся значение текущего преобразования АЦП. При
подключении рабочего канала АЦП на вывод GND и вывод +3V3 наблюдаются соответствующие
изменения в выводе данных (близкое к нулю для GND, близкое к 4096 для +3V3, допускается
вывод в виде значения напряжения).
4. Нажатия на пользовательскую кнопку обрабатываются корректно и однозначно определяют
состояние светодиода, многократные срабатывания не происходят.
5. При однократном нажатии на кнопку светодиод переключает своё состояние, при этом не
нарушается вывод сообщений о преобразовании АЦП.

Замечания
1. Решение оформляется проектом с именем «Task_5». В начале файла «main.c» должен идти блок
комментариев, в котором приводятся полные условия задания и задействованные в решении
аппаратные ресурсы демо-платы (кнопки, светодиоды, выводы портов связи).
2. Требований к выбору схемы управления памятью не предъявляется.
3. Требуется самостоятельно определить приоритет всех трёх задач.
4. Чтение пользовательской кнопки реализовать в виде отдельной задачи без использования
прерываний, но с использованием механизма программного антидребезга.
5. При реализации задачи работы с АЦП:
5.1. Использовать режим одиночного преобразования без прерываний,
5.2. Период вызова задачи задать через vTaskDelayUntil.


USING STM32F407VET6 board:

SWD interface
PE10 input key S1
PE13 output LED1
PA5 analog input ADC1, external potentiometr 10kOm
*/
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "gpio.h"
#include "usart_dbg.h"
#include "adc.h"

#include <stdio.h>
#include <stdlib.h>

/* Typedef for ID field in new struct */
typedef enum {
  MSG_ID_KEYBOARD = 0,
  MSG_ID_ADC
} msgID;
/* Typedef for special struct - element of queue */
typedef struct {
  msgID id;
  uint16_t msgVal;
} queueMsg;

/************************global values********************************/
#define QUEUE_REC_NUM (100) //record number in queue
#define QUEUE_TIMEOUT_SEND (500) //timeout write in queue
#define QUEUE_TIMEOUT_REC (1000) //timeout read in queue

/* Queue handler */
QueueHandle_t myQueue;
/****************************func************************************/

/* read user button */
void vTaskButton(void * pvParameters){
  portTickType lastTickCountTaskButton = xTaskGetTickCount();
  queueMsg senderMsg;
  while(1){
    if(!(GPIOE->IDR & GPIO_IDR_ID10)){
      vTaskDelayUntil(&lastTickCountTaskButton, 100);
      senderMsg.id = MSG_ID_KEYBOARD;
      senderMsg.msgVal = 1;
      xQueueSend(myQueue, &senderMsg, QUEUE_TIMEOUT_SEND); 
      while(!(GPIOE->IDR & GPIO_IDR_ID10)){}    
    }
  }
  vTaskDelete(NULL);
}
/* Measurement ADC */
void vTaskADC(void * pvParameters){
  portTickType lastTickCountTask2 = xTaskGetTickCount();
  queueMsg senderMsg;
  while(1){
    ADC1->CR2 |= ADC_CR2_JSWSTART;
    while(ADC1->SR & ADC_SR_JEOC){}
    senderMsg.id = MSG_ID_ADC;
    senderMsg.msgVal = (uint16_t)ADC1->JDR1;
    xQueueSend(myQueue, &senderMsg, QUEUE_TIMEOUT_SEND);
    ADC1->SR &=~(ADC_SR_JEOC);
    vTaskDelayUntil(&lastTickCountTask2, 500);
  }
  vTaskDelete(NULL);
}
/* Read queue */
void vTaskControl(void * pvParameters){
  queueMsg receiveMsg;
  uint16_t usMesVolt = 0;
  while(1){
    if(xQueueReceive(myQueue, &receiveMsg, QUEUE_TIMEOUT_REC) == pdTRUE){
      switch (receiveMsg.id) {
        case MSG_ID_KEYBOARD:
          if(GPIOE->ODR & GPIO_ODR_OD13){
            GPIOE->BSRR |= GPIO_BSRR_BR13;
          }else{
            GPIOE->BSRR |= GPIO_BSRR_BS13;
          }
          break;
        case MSG_ID_ADC:
          usMesVolt = (uint16_t)(receiveMsg.msgVal * 3300 / 4095);
          printf("ADC = %dmV\n",usMesVolt);
          break;
        default:
          break;
      }
    }
  }
  vTaskDelete(NULL);
}
/****************************main************************************/

 int main(void) {
  GPIO_init();
  usart1_init();
  ADC1_Init();
  printf("--- System startup ---\n");
  
  myQueue = xQueueCreate(QUEUE_REC_NUM, sizeof(queueMsg));

  xTaskCreate(vTaskButton, "vTaskButton", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(vTaskADC, "vTaskADC", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
  xTaskCreate(vTaskControl, "vTaskControl", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
  vTaskStartScheduler();
  while(1){}
}

/*************************** End of file ****************************/
