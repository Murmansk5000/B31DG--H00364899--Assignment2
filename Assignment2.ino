#include <Arduino.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>


// task 1
const int digitalSignalPin1 = 27;

// task2 and task 3
const int frequencyPin2 = 25;  // For task 2
const int frequencyPin3 = 26;  // For task 3

struct Frequencies {
  int task2Frequency;
  int task3Frequency;
};
Frequencies freqData;

SemaphoreHandle_t freqSemaphore;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long lastRiseTime2 = 0;
volatile unsigned long lastRiseTime3 = 0;

volatile int frequency2 = 0;
volatile int frequency3 = 0;
unsigned long period2 = 0;
unsigned long period3 = 0;

volatile unsigned long lastInterruptTime2 = 0;
volatile unsigned long lastInterruptTime3 = 0;
const unsigned long debounceTime2 = 100;
const unsigned long debounceTime3 = 100;



// task 4
const int analogInputPin4 = 35;
const int ledPin4 = 21;

int readings[10] = { 0 };
int readIndex = 0;  // Index of current number
float average = 0;

// task 5


// task 7
const int ledPin7 = 19;
const int buttonPin7 = 5;
// debounce
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 100;
int buttonState = LOW;
int lastButtonState = LOW;


// task 8

// Task function prototypes
void task1(void *pvParameters);
void task2(void *pvParameters);
void task3(void *pvParameters);
void task4(void *pvParameters);
void task5(void *pvParameters);
void task7_1(void *pvParameters);
void task7_2(void *pvParameters);
void task8(void *pvParameters);
void IRAM_ATTR onRise2();
void IRAM_ATTR onRise3();


void setup(void) {
  freqSemaphore = xSemaphoreCreateMutex();
  Serial.begin(9600);

  pinMode(digitalSignalPin1, OUTPUT);
  pinMode(frequencyPin2, INPUT);
  pinMode(frequencyPin3, INPUT);
  pinMode(ledPin4, OUTPUT);
  pinMode(analogInputPin4, INPUT);
  pinMode(ledPin7, OUTPUT);
  pinMode(buttonPin7, INPUT);

  xTaskCreate(task1, "DigitalSignalOutput", 1024, NULL, 5, NULL);
  xTaskCreate(task2, "FrequencyMeasure1", 1024, NULL, 5, NULL);
  xTaskCreate(task3, "FrequencyMeasure2", 1024, NULL, 5, NULL);
  xTaskCreate(task4, "SampleAnalogInput", 1024, NULL, 3, NULL);
  xTaskCreate(task5, "LogToSerialPort", 1600, NULL, 3, NULL);
  xTaskCreate(task7_1, "checkButton", 1024, NULL, 4, NULL);
  xTaskCreate(task7_2, "controlLED", 1024, NULL, 2, NULL);
  xTaskCreate(task8, "CPUWork", 1024, NULL, 1, NULL);
}



void loop() {
}


UBaseType_t uxHighWaterMark;
uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
Serial.print("Stack for task: ");
Serial.println(uxHighWaterMark);

// Task1
void task1(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    digitalWrite(digitalSignalPin1, HIGH);
    ets_delay_us(180);

    digitalWrite(digitalSignalPin1, LOW);
    ets_delay_us(40);

    digitalWrite(digitalSignalPin1, HIGH);
    ets_delay_us(530);

    digitalWrite(digitalSignalPin1, LOW);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));
  }
}



void IRAM_ATTR onRise2() {
  unsigned long currentTime = micros();

  if (currentTime - lastInterruptTime2 > debounceTime2) {  // debounce
    lastInterruptTime2 = currentTime;
    if (lastRiseTime2 != 0) {                 // Ensure it's not the first interrupt
      period2 = currentTime - lastRiseTime2;  //  calculate period
    }
    lastRiseTime2 = currentTime;
  }
}

void IRAM_ATTR onRise3() {
  unsigned long currentTime = micros();
  if (currentTime - lastInterruptTime3 > debounceTime3) {  // debounce
    lastInterruptTime3 = currentTime;
    if (lastRiseTime3 != 0) {                 // Ensure it's not the first interrupt
      period3 = currentTime - lastRiseTime3;  //  calculate period
    }
    lastRiseTime3 = currentTime;
  }
}

void task2(void *parameter) {
  attachInterrupt(digitalPinToInterrupt(frequencyPin2), onRise2, RISING);

  for (;;) {
    if (xSemaphoreTake(freqSemaphore, (TickType_t)10) == pdTRUE) {
      if (period2 != 0) {
        frequency2 = 1000000 / period2;  // frequency calculation to task
      }
      freqData.task2Frequency = frequency2;
      xSemaphoreGive(freqSemaphore);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void task3(void *parameter) {
  attachInterrupt(digitalPinToInterrupt(frequencyPin3), onRise3, RISING);

  for (;;) {
    if (xSemaphoreTake(freqSemaphore, (TickType_t)10) == pdTRUE) {
      if (period3 != 0) {
        frequency3 = 1000000 / period3;  // frequency calculation to task
      }
      freqData.task3Frequency = frequency3;
      xSemaphoreGive(freqSemaphore);
    }
    vTaskDelay(pdMS_TO_TICKS(8));
  }
}





void task4(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize the last wake time for the task
  const TickType_t xFrequency = pdMS_TO_TICKS(20);  // Set the task frequency to 20 milliseconds

  while (1) {
    int readValue = analogRead(analogInputPin4);  //Read from pin

    // Update the moving average, remove the oldest one and add the new one
    average -= readings[readIndex] / 10.0;
    readings[readIndex] = readValue;
    average += readings[readIndex] / 10.0;

    readIndex = (readIndex + 1) % 10;  // Advance the index for the next reading

    // Serial.println(average); // For test
    // Control the LED
    if (average > 2048) {
      digitalWrite(ledPin4, HIGH);
    } else {
      digitalWrite(ledPin4, LOW);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Delay until the next cycle
  }
}


// Task 5: Logs the frequency measurements for Task 2 and Task 3 after scaling the values
void task5(void *parameter) {
  while (1) {
    if (xSemaphoreTake(freqSemaphore, (TickType_t)10) == pdTRUE) {
      // Scale the frequency from Task 2 within a range of 0 to 99
      int scaledFrequency2 = map(freqData.task2Frequency, 333, 1000, 0, 99);
      scaledFrequency2 = constrain(scaledFrequency2, 0, 99);  // Constrain values to ensure they are within the range

      // Scale the frequency from Task 3 similarly
      int scaledFrequency3 = map(freqData.task3Frequency, 500, 1000, 0, 99);
      scaledFrequency3 = constrain(scaledFrequency3, 0, 99);  // Constrain values

      // Print the frequencies to the Serial monitor
      // Serial.printf("%d,%d\n", freqData.task2Frequency, freqData.task3Frequency); //For test
      Serial.printf("%d,%d\n", scaledFrequency2, scaledFrequency3);

      xSemaphoreGive(freqSemaphore);  // Release the semaphore after processing
    }
    vTaskDelay(pdMS_TO_TICKS(200));  // Delay the task for 200 milliseconds
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


// Task 7
void task7_1(void *pvParameters) {
  while (true) {
    int reading = digitalRead(buttonPin7);  // Read the current button state

    // If the current reading is different from the last state, reset the time
    if (reading != lastButtonState) {
      lastDebounceTime = millis();
    }

    // Time over threshold, considered valid
    if ((millis() - lastDebounceTime) > debounceDelay) {
      // Update button status
      if (reading != buttonState) {
        buttonState = reading;
        // Serial.println("Button is Pressed"); // For test
      }
    }

    // Save the current state for next time
    lastButtonState = reading;
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void task7_2(void *pvParameters) {
  int lastControlState = LOW;  // Last state of LED
  for (;;) {
    // Check if the button has just been pressed (different state than last time)
    if (buttonState == HIGH && lastControlState != buttonState) {
      digitalWrite(ledPin7, !digitalRead(ledPin7));  // Change the state of the LEDs
      lastControlState = buttonState;                // Update the last known state
    } else if (buttonState != lastControlState) {
      lastControlState = buttonState;  // Update lastControlState without toggling the LED
    }
  }
}

void CPU_work(int time) {
  // Simulation of CPU, assume cpu work time
  unsigned long startTime = millis();
  unsigned long endTime = startTime + time;
  // Loop until current time reaches or exceeds target end time
  while (millis() < endTime) {
  }
}

// Task 8
void task8(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;  // Convert 20ms to tick period

  // Initialize the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    CPU_work(2);
    // Wait for the next cycle.
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}