#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

SemaphoreHandle_t freq_data_mutex; 
// Task1:
const int task1_pin = 12;//Pins for task 1
// Task2&3&5:
const int SIGNAL_PIN_2 = 15;
const int SIGNAL_PIN_3 = 16;
// Variables to track Task 2 signal state and frequency
volatile int lastValue_task2 = LOW; // Previous signal state
volatile unsigned long firstChangeTime_task2 = 0; // Time of the first signal state change
volatile unsigned long secondChangeTime_task2 = 0; // Time of the second signal state change
volatile boolean firstChangeDetected_task2 = false; // Flag indicating the first signal state change detection
volatile int lastFrequency_task2 = 0; // Last measured frequency for Task 2
// Variables to track Task 3 signal state and frequency
volatile int lastValue_task3 = LOW; 
volatile unsigned long firstChangeTime_task3 = 0; 
volatile unsigned long secondChangeTime_task3 = 0; 
volatile boolean firstChangeDetected_task3 = false; 
volatile int lastFrequency_task3 = 0; // Last measured frequency for Task 3
unsigned long elapsedTime = 0;// Variable to store the elapsed time between signal state changes
// Define a structure to store the frequencies measured by Task #2 and Task #3
typedef struct {
  int freq_task2;
  int freq_task3;
} FrequencyData;
FrequencyData freq_data;// Global variable to store the frequencies

//Task4:
#define ANALOG_INPUT_PIN 27
#define LED4_PIN 19
#define MAX_ANALOG_READING 1023 
#define MAX_ANALOG_VOLTAGE 3.3 
#define HALF_MAX_RANGE_VOLTAGE (MAX_ANALOG_VOLTAGE / 2.0) 
#define SAMPLES_COUNT 10 // Specify the number of samples to be stored for calculating the running average
double samples[SAMPLES_COUNT]; // Array to store the most recent analog readings for computing the running average
int samplesIndex = 0; // Index of the current sample in the samples array
int analogReading = 0;
double analogVoltage = 0;
double runningAverage = 0;

// Task 7:
const int LED7_PIN = 23;
const int BUTTON_PIN = 25;
bool ledState = LOW;
bool buttonState = LOW;
bool lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
// Create an event queue
QueueHandle_t eventQueue;
// Define event types
typedef enum {
  BUTTON_PRESSED
} EventType;
// Define the structure for events
typedef struct {
  EventType type;
} Event;



// Task 1: Output Digital Signal
// This task generates a digital signal on the specified pin (task1_pin).
// It toggles the pin high and low with specific delays to create the desired signal.
void taskOutputDigitalSignal(void *pvParameters) {
  pinMode(task1_pin, OUTPUT);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;){
    digitalWrite(task1_pin, HIGH);
    delayMicroseconds(180);
    digitalWrite(task1_pin, LOW);
    delayMicroseconds(40);
    digitalWrite(task1_pin, HIGH);
    delayMicroseconds(530);
    digitalWrite(task1_pin, LOW);
    vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(4));
  }
}


// Task 2: Measure frequency and update freq_data
// This task measures the frequency of a signal on a specified pin (SIGNAL_PIN_2) using interrupts.
// It updates the freq_data structure with the measured frequency of Task 2.
void taskMeasureFrequency2(void *pvParameters) {
  pinMode(SIGNAL_PIN_2, INPUT_PULLUP);
  // Calculates the current frequency using interrupt routines. 
  attachInterrupt(digitalPinToInterrupt(SIGNAL_PIN_2), handleInterrupt_task2, CHANGE); 
  for(;;){
    // Obtain the semaphore to protect access to freq_data
    if (xSemaphoreTake(freq_data_mutex, portMAX_DELAY) == pdTRUE) {
      freq_data.freq_task2 = lastFrequency_task2;
      xSemaphoreGive(freq_data_mutex); // Release the semaphore
    } 
     vTaskDelay(pdMS_TO_TICKS(20)); // Delay 20 milliseconds
  }
  
}

// Task 3: Measure frequency and update freq_data
// This task measures the frequency of a signal on a specified pin (SIGNAL_PIN_3) using interrupts.Use same fuction as Task2
void taskMeasureFrequency3(void *pvParameters) {
  pinMode(SIGNAL_PIN_3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_PIN_3), handleInterrupt_task3, CHANGE); 
  for(;;){
    // Obtain the semaphore to protect access to freq_data
    if (xSemaphoreTake(freq_data_mutex, portMAX_DELAY) == pdTRUE) {
      freq_data.freq_task3 = lastFrequency_task3;
      xSemaphoreGive(freq_data_mutex); // Release the semaphore
     } 
    vTaskDelay(pdMS_TO_TICKS(8)); 
  }
  
}

// Task 5: Print frequencies stored in freq_data to serial port
// This task prints the frequencies stored in the freq_data structure to the serial port.
// It scales the frequencies and ensures they are bounded between 0 and 99.
void taskLogFrequencies5(void *pvParameters) {
  for (;;) {
    // Obtain the semaphore to protect access to freq_data
    if (xSemaphoreTake(freq_data_mutex, portMAX_DELAY) == pdTRUE) {
      //step1. Get frequencies of serial port
      int scaled_freq_task2 = map(freq_data.freq_task2, 333, 1000, 0, 99);
      int scaled_freq_task3 = map(freq_data.freq_task3, 500, 1000, 0, 99);
      xSemaphoreGive(freq_data_mutex); // Release the semaphore
      //step2. Ensures the freq bounded between 0 and 99.
      if (freq_data.freq_task2 <= 333) { scaled_freq_task2 = 0;
      } else if (freq_data.freq_task2 >= 1000) { scaled_freq_task2 = 99; } 

      if (freq_data.freq_task3 <= 500) { scaled_freq_task3 = 0;
      } else if (freq_data.freq_task3 >= 1000) {  scaled_freq_task3 = 99; } 
      //step3.Print frequencies to serial port
      Serial.print("Frequency Task: ");
      Serial.print(scaled_freq_task2);
      Serial.print(",");
      Serial.println(scaled_freq_task3);
    }
    vTaskDelay(pdMS_TO_TICKS(200)); // Delay for 200ms
  }
}

// Interrupt handler for Task 2
// This function is called whenever an interrupt is triggered on the pin associated with Task 2 (SIGNAL_PIN_2).
// It measures the frequency of the signal and updates the relevant variables.
void ICACHE_RAM_ATTR handleInterrupt_task2() {
  int newValue = digitalRead(SIGNAL_PIN_2); 
  if (newValue != lastValue_task2) { // If the current state is different from the previous state
    if (!firstChangeDetected_task2) { // If it's the first time the state has changed
      // Set the first change flag to true and record the time of the first state change
      firstChangeDetected_task2 = true; 
      firstChangeTime_task2 = micros(); 
    } else { // If it's the second time the state has changed
      // Record the time of the second state change and calculate the time interval between two state changes
      secondChangeTime_task2 = micros(); 
      elapsedTime = secondChangeTime_task2 - firstChangeTime_task2; 
      // Calculate and store frequency
      lastFrequency_task2 = 500000 / elapsedTime;
      firstChangeDetected_task2 = false; // Reset the first change flag
    }
  }
  lastValue_task2 = newValue; // Update the previous state
}

// Interrupt handler for Task 3
// This function is called whenever an interrupt is triggered on the pin associated with Task 3 (SIGNAL_PIN_3).
// It measures the frequency of the signal and updates the relevant variables.
void ICACHE_RAM_ATTR handleInterrupt_task3() {
  int newValue = digitalRead(SIGNAL_PIN_3); 
  if (newValue != lastValue_task3) { 
    if (!firstChangeDetected_task3) { 
      firstChangeDetected_task3 = true; 
      firstChangeTime_task3 = micros();
    } else {
      secondChangeTime_task3 = micros(); 
      unsigned long elapsedTime = secondChangeTime_task3 - firstChangeTime_task3; 
      lastFrequency_task3 = 500000.0 / elapsedTime;
      firstChangeDetected_task3 = false; 
    }
  }
  lastValue_task3 = newValue; 
}


// Task 7: Handle Button Events
// This task monitors the state of a button connected to BUTTON_PIN.
// It debounces the button to prevent false triggers and sends an event
// to the event queue when the button is pressed.
void taskButtonEvent7(void *pvParameters) {
  (void) pvParameters; 
  pinMode(LED7_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  for (;;) {
    int reading = digitalRead(BUTTON_PIN);

    // step1.Perform button debounce
    if (reading != lastButtonState) {
      lastDebounceTime = millis();
    }
    //step2.After debounce judge the state of button
    if ((millis() - lastDebounceTime) > debounceDelay) {
      // Update button state
      if (reading != buttonState) {
        buttonState = reading;
        // Send an event to the event queue when the button is pressed
        if (buttonState == HIGH) {
          Event event = {BUTTON_PRESSED};
          xQueueSend(eventQueue, &event, portMAX_DELAY);
        }
      }
    }
    // step3.Update the lastButtonState
    lastButtonState = reading;
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}


// Task 7: Handle LED Events
// This task listens for events from the event queue and toggles the LED state
// when a BUTTON_PRESSED event is received.
void taskLEDEvent7(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    // Receive events from the event queue
    Event event;
    if (xQueueReceive(eventQueue, &event, portMAX_DELAY) == pdPASS) {
      // Toggle the LED state when a BUTTON_PRESSED event is received
      if (event.type == BUTTON_PRESSED) {
        ledState = !ledState;
        digitalWrite(LED7_PIN, ledState);
        Serial.println("LED toggled!");
      }
    }
  }
}

// Task 4: Read Analog Input and Compute Running Average
// This task reads analog input from a sensor connected to ANALOG_INPUT_PIN.
// It calculates the running average of the analog readings and checks if the
// average value exceeds half of the maximum voltage range. If it does, it
// turns on the LED connected to LED4_PIN to indicate an error condition.
void taskAnalogReading4(void *pvParameters) {
  pinMode(ANALOG_INPUT_PIN, INPUT);
  pinMode(LED4_PIN, OUTPUT);

  for (;;) {
    analogReading = analogRead(ANALOG_INPUT_PIN);
    // Convert analog reading to voltage
    analogVoltage = (analogReading / (double)MAX_ANALOG_READING) * MAX_ANALOG_VOLTAGE;

    // Step1.Update running average
    samples[samplesIndex] = analogVoltage;
    samplesIndex = (samplesIndex + 1) % SAMPLES_COUNT;
    runningAverage = 0;
    for (int i = 0; i < SAMPLES_COUNT; i++) {
      runningAverage += samples[i];
    }
    runningAverage /= SAMPLES_COUNT;

    // step2.If running average exceeds half of the maximum range voltage, indicate error
    if (runningAverage > HALF_MAX_RANGE_VOLTAGE) {
      digitalWrite(LED4_PIN, HIGH); 
    } else {
      digitalWrite(LED4_PIN, LOW); 
    }
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}

void CPU_work(int time) {
  unsigned long endTime = millis() + time;
  for (; millis() - endTime <0;) { 
  }
}

// Define a periodic task that calls CPU_work(2) every 20ms
void periodicTask(void *pvParameters) {
  for (;;) {
    // Let the CPU work for approximately 2ms
    CPU_work(2); 
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup(void) {
  Serial.begin(9600);
  eventQueue = xQueueCreate(10, sizeof(Event));

  freq_data_mutex = xSemaphoreCreateMutex();
  // Create and start tasks
  xTaskCreatePinnedToCore(taskOutputDigitalSignal, "Task1", 1024, NULL, 4, NULL, 0);
  xTaskCreate(taskMeasureFrequency2, "Task2", 2048, NULL, 3, NULL);
  xTaskCreate(taskMeasureFrequency3, "Task3", 2048, NULL, 3, NULL);
  xTaskCreate(taskLogFrequencies5, "Task5", 1024, NULL, 1, NULL);
  xTaskCreate(taskAnalogReading4, "Task4", 1024, NULL, 2, NULL);
  xTaskCreate(taskLEDEvent7, "LEDEvent", 1000, NULL, 2, NULL);
  xTaskCreate(taskButtonEvent7, "ButtonEvent", 1024, NULL, 2, NULL);
  xTaskCreate(periodicTask, "Task8",1024, NULL, 0,NULL);   
}

void loop() {
  
}