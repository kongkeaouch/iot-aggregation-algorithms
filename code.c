#include "contiki.h"
#include <stdio.h>             /* For printf() */
#include <stdlib.h>            /* For malloc */
#include "dev/button-sensor.h" // For button
#include "dev/leds.h"          // For LED
#include "dev/light-sensor.h"

#define BUFFER_SIZE 12        // Number of readings stored in buffer \
                              // also represent high segment size as \
                              // 12 = 12/1, buffer size is 12 and no aggregation uses 1-into-1
#define LOW_SEGMENT_SIZE 1    // 1 = 12/12, buffer size is 12 and low aggregation uses 12-into-1
#define MEDIUM_SEGMENT_SIZE 3 // 3 = 12/4, buffer size is 12 and medium aggregation uses 4-into-1
#define MEDIUM_AGGREGATION_STDDEV 10
#define HIGH_AGGREGATION_STDDEV 100
#define SAX_ALPHABBET_SIZE 6 // abcdef, cutlines = [-0.97 -0.43 0 0.43 0.97]
#define SMOOTHING_FACTOR 0.7

/*---------------------------------------------------------------------------*/
/* Interface of functions */

// Utility functions
int d1(float f);
unsigned int d2(float f);
float squareRoot(float S);
float mean(float *data, int size);

// Core functions
float getLight(void);
float getStdDev(float *B);
int getW(float StdDev);
int getSaxW(float StdDev);
float *getPAA(float *data, float *PAA, int W);
int getSizeRLE(float *B);
float *getRLE(float *B, float *RLE);
char getSAXLetter(float value);
float *getSaxPAA(float *B, float *SaxPAA, int SaxW, float StdDev);
char *getSAX(float *SaxPAA, char *SAX, int SaxW);
float *getEMA(float *B, float *EMA);

// Output functions
void printBuffer(float *B);
void printStdDev(float StdDev);
void printAggregation(int W);
void printX(float *X, int W);
void printRLE(float *RLE, int SizeRLE);
void printSaxPAA(float *SaxPAA, int SaxW);
void printSmoothingFactor();
void printEMA(float *EMA);
void printSAX(char *SAX, int W);
void flashActivityLED(float StdDev);

/*---------------------------------------------------------------------------*/
PROCESS(aggregation, "Aggregation");
AUTOSTART_PROCESSES(&aggregation);
/*---------------------------------------------------------------------------*/
/* Implementation of the main process */
PROCESS_THREAD(aggregation, ev, data)
{
  static struct etimer timer;
  static float *B;  // Buffer
  static float *B2; // Buffer
  // static float ema_mock[12] = {10, 2, 12, 2, 19, 1, 19, 100, 6, 4, 66, 33};

  static float StdDev;  // Standard deviation
  static float *X, *X2; // Core aggregation
  static float *EMA;
  static float *RLE;     // Run-length aggregation
  static char *SAX;      // Symbolic Aggregate Approximation (SAX) aggregation
  static float *SAX_PAA; // PAA Vallues before becoming SAX letters
  static int W, SaxW;    // Number of segments, each of is the average of its data points
                         // used by Piecewise Aggregate Approximation (PAA) to reduce dimensions
  static int count;      // Buffer count
  static int SizeRLE;
  static int led_on, led_timer;
  float measure; // Light measure regenerated every event

  PROCESS_BEGIN();
  SENSORS_ACTIVATE(light_sensor);
  SENSORS_ACTIVATE(button_sensor);
  leds_off(LEDS_ALL);

  // Initialise variables and allocate memory for buffer
  count = 0;
  SizeRLE = 0;
  W = 0;
  SaxW = 0;
  measure = 0.0;
  StdDev = 0.0;
  led_timer = 0;
  B = (float *)malloc(BUFFER_SIZE * sizeof(float));

  // Set the etimer module to generate an event in 0.5 second
  etimer_set(&timer, CLOCK_CONF_SECOND / 2);

  while (1)
  {
    PROCESS_WAIT_EVENT();

    if (ev == PROCESS_EVENT_TIMER)
    {
      // Get light measurement and convert it to lumens
      measure = 1.0 * d1(getLight());
      printf("reading = %d\n", d1(measure));

      // Assign measurement to buffer & count the measures
      B[count] = measure;
      count++;

      // Perform aggregation process every 12 readings
      if (count == BUFFER_SIZE)
      {
        // Free up memory of previous aggregations to take in new ones
        // free(X);
        free(EMA);
        // free(X2);
        // free(RLE);
        // free(SAX);

        // Calculate standard deviation
        // StdDev = getStdDev(B);

        // Get number of segments based on
        // level of standard deviation (low, medium, high)
        // W = getW(StdDev);

        // SaxW = getSaxW(StdDev);
        // Get RLE size by count each repetitive stream as one value
        // SizeRLE = getSizeRLE(B);

        // Allocate memory for aggregation values
        // Size of X and SAX depend on W
        // X = (float *)malloc(W * sizeof(float));
        EMA = (float *)malloc(BUFFER_SIZE * sizeof(float));
        // RLE = (float *)malloc(SizeRLE * sizeof(float));
        // SAX_PAA = (float *)malloc(SaxW * sizeof(float));
        // SAX = (char *)malloc(SaxW * sizeof(char));

        // Compute aggregations
        // X = getPAA(B, X, W); // Use PAA as we need to average data points into each of W segments

        // RLE = getRLE(B, RLE);
        // SAX_PAA = getSaxPAA(B, SAX_PAA, SaxW, StdDev);
        // SAX = getSAX(SAX_PAA, SAX, SaxW);
        EMA = getEMA(B, EMA);

        // Print outputs of main features
        // printStdDev(StdDev);
        // printAggregation(W);
        // printX(X, W);

        printBuffer(B);
        printSmoothingFactor();
        printEMA(EMA);        

        // Print outputs of extra features
        // printf("\nExtra features\n");
        // printRLE(RLE, SizeRLE);
        // printSaxPAA(SAX_PAA, SaxW);
        // printSAX(SAX, SaxW);

        // Reset count
        count = 0;
      }

      if (led_timer > 0)
      {
        led_timer--;

        if (led_timer == 0)
        {
          leds_off(LEDS_ALL);
        }
      }

      // Reset the timer so it will generate another event
      etimer_reset(&timer);
    }
    else if (ev == sensors_event && data == &button_sensor)
    {
      leds_off(LEDS_ALL);
      flashActivityLED(StdDev);
      led_timer = 10;
    }
  }

  // Free up memory once the work is done
  free(B);
  free(X);
  free(X2);
  free(RLE);
  free(SAX);

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/* Implementation of functions */

// Utility functions
int d1(float f) // Integer part
{
  return ((int)f);
}

unsigned int d2(float f) // Fractional part
{
  if (f > 0)
    return (100 * (f - d1(f)));
  else
    return (100 * (d1(f) - f));
}

float squareRoot(float S)
{
  float x = S;    // initial guess
  float y = 1;    //
  float e = 0.01; /* accuracy level */

  // This Babylonian method can be derived from (but predates) Newton–Raphson method.
  // Get the next approximation for root using average of x and y
  // until desired approximation is achieved.
  while (x - y > e)
  {
    x = 0.5 * (x + y);
    y = S / x;
  }
  return x;
}

float mean(float *data, int size)
{
  float sum = 0.0;

  int i;
  for (i = 0; i < size; i++)
    sum += data[i];

  return sum / size;
}

// Core functions
float getLight(void)
{
  float V_sensor = 1.5 * light_sensor.value(LIGHT_SENSOR_PHOTOSYNTHETIC) / 4096;
  // ^ ADC-12 uses 1.5V_REF
  float I = V_sensor / 100000;             // xm1000 uses 100kohm resistor
  float light_lx = 0.625 * 1e6 * I * 1000; // convert from current to light intensity
  return light_lx;
}

float getStdDev(float *B)
{
  float meanB = mean(B, BUFFER_SIZE),
        varsum = 0.0,
        variance = 0.0;
  int i;

  // Find variance and its square root
  // then we got standard deviation
  for (i = 0; i < BUFFER_SIZE; i++)
  {
    varsum += (B[i] - meanB) * (B[i] - meanB);
  }

  variance = varsum / BUFFER_SIZE;

  return squareRoot(variance);
}

int getW(float StdDev)
{
  int W = 0;

  if (StdDev < MEDIUM_AGGREGATION_STDDEV)
  {
    W = LOW_SEGMENT_SIZE;
  }
  else if (StdDev < HIGH_AGGREGATION_STDDEV)
  {
    W = MEDIUM_SEGMENT_SIZE;
  }
  else
  {
    // High segment size = buffer size, no aggregation
    W = BUFFER_SIZE;
  }

  return W;
}

int getSaxW(float StdDev)
{
  int sax_w = 0;

  if (StdDev < MEDIUM_AGGREGATION_STDDEV)
  {
    sax_w = 3;
  }
  else if (StdDev < HIGH_AGGREGATION_STDDEV)
  {
    sax_w = 6;
  }
  else
  {
    // High segment size = buffer size, no aggregation
    sax_w = BUFFER_SIZE;
  }

  return sax_w;
}

float *getPAA(float *data, float *PAA, int W)
{
  // Number of data points to be averaged
  int windowSize = BUFFER_SIZE / W, i, j;

  // For every segment, sum up and average data points to reduce size
  for (i = 0; i < W; i++)
  {
    float sum = 0.0;

    /* Amendment to original submission */
    /* Change from j = i to j = i * windowSize */
    /* Critical mistake that resulted in wrong X result when W=4 and wrong SAX result when SaxW=3/6 */
    for (j = i * windowSize; j < (i + 1) * windowSize; j++) // for (j = i; j < (i + 1) * windowSize; j++) (original submission) //
    {
      sum += data[j];
    }

    PAA[i] = sum / windowSize;
  }

  return PAA;
}

int getSizeRLE(float *B)
{
  int i, SizeRLE = 0, prev = 0;

  for (i = 0; i < BUFFER_SIZE; i++)
  {
    if (B[i] != prev)
    {
      prev = B[i];
      SizeRLE++;
    }
  }

  return SizeRLE;
}

float *getRLE(float *B, float *RLE)
{
  int i, rle_i = 0, prev = 0;

  // Continuosly store new value that is not same as previous one
  for (i = 0; i < BUFFER_SIZE; i++)
  {
    if (B[i] != prev)
    {
      prev = B[i];
      RLE[rle_i] = prev;
      rle_i++;
    }
  }

  return RLE;
}

char getSAXLetter(float value)
{
  char letter;

  // Cutlines based on alphabet size of 6
  if (value < -0.97)
  {
    letter = 'a';
  }
  else if (value < -0.43)
  {
    letter = 'b';
  }
  else if (value < 0)
  {
    letter = 'c';
  }
  else if (value < 0.43)
  {
    letter = 'd';
  }
  else if (value < 0.97)
  {
    letter = 'e';
  }
  else
  {
    letter = 'f';
  }

  return letter;
}

float *getSaxPAA(float *B, float *SaxPAA, int SaxW, float StdDev)
{
  float meanB = mean(B, BUFFER_SIZE),
        *normalizedB = (float *)malloc(BUFFER_SIZE * sizeof(float)),
        *PAA = (float *)malloc(SaxW * sizeof(float));
  int i;

  // Normalize buffer elements
  for (i = 0; i < BUFFER_SIZE; i++)
  {
    normalizedB[i] = (B[i] - meanB) / StdDev;
  }

  // Reduce size of norm values by W segments
  SaxPAA = getPAA(normalizedB, PAA, SaxW);

  // Always deallocate memory that is no longer needed
  free(normalizedB);

  return SaxPAA;
}

char *getSAX(float *SaxPAA, char *SAX, int SaxW)
{
  // Assign appropriate letter based on SAX breakpoints
  int i;
  for (i = 0; i < SaxW; i++)
  {
    SAX[i] = getSAXLetter(SaxPAA[i]);
  }

  return SAX;
}

float *getEMA(float *B, float *EMA)
{
  int i;

  for (i = 0; i < BUFFER_SIZE; i++)
  {
    if (i == 0)
    {
      EMA[i] = B[i];
      continue;
    }

    EMA[i] = (SMOOTHING_FACTOR * B[i]) + ((1 - SMOOTHING_FACTOR) * EMA[i - 1]);
  }

  return EMA;
}

// Output functions
void printBuffer(float *B)
{
  printf("B = [");

  int i;
  for (i = 0; i < BUFFER_SIZE; i++)
  {
    printf("%d", d1(B[i]));

    if (i < BUFFER_SIZE - 1)
    {
      printf(", ");
    }
  }

  printf("]\n");
}

void printStdDev(float StdDev)
{
  printf("StdDev = %d.%02u\n", d1(StdDev), d2(StdDev));
}

void printAggregation(int W)
{
  switch (W)
  {
  case BUFFER_SIZE:
    printf("Aggregation = no aggregation\n");
    break;

  case MEDIUM_SEGMENT_SIZE:
    printf("Aggregation = 4-into-1\n");
    break;

  case LOW_SEGMENT_SIZE:
    printf("Aggregation = 12-into-1\n");
    break;
  }
}

void printFloatArray(float *data, int size, char title[])
{
  printf("%s = [", title);

  int i;
  for (i = 0; i < size; i++)
  {
    if (d1(data[i]) == 0 && data[i] < 0)
    {
      printf("-");
    }

    printf("%d.%02u", d1(data[i]), d2(data[i]));

    if (i < size - 1)
    {
      printf(", ");
    }
  }

  printf("]\n");
}

void printIntArray(float *data, int size, char title[])
{
  printf("%s = [", title);

  int i;
  for (i = 0; i < size; i++)
  {
    printf("%d", d1(data[i]));

    if (i < size - 1)
    {
      printf(", ");
    }
  }

  printf("]\n");
}

void printX(float *X, int W)
{
  if (W != BUFFER_SIZE)
  {
    printFloatArray(X, W, "X");
  }
  else
  {
    printIntArray(X, W, "X");
  }
}

void printRLE(float *RLE, int SizeRLE)
{
  printIntArray(RLE, SizeRLE, "RLE");
}

void printSaxPAA(float *SaxPAA, int SaxW)
{
  printFloatArray(SaxPAA, SaxW, "SAX_PAA");
}

void printSmoothingFactor()
{
  printf("\nSmoothing Factor = %d.%01u\n", d1(SMOOTHING_FACTOR), d2(SMOOTHING_FACTOR));
}

void printEMA(float *EMA)
{
  printFloatArray(EMA, BUFFER_SIZE, "EMA");
}

void printSAX(char *SAX, int SaxW)
{
  printf("SAX = ");

  int i;
  for (i = 0; i < SaxW; i++)
  {
    printf("%c", SAX[i]);
  }

  printf("\n\n");
}

void flashActivityLED(float StdDev)
{
  if (StdDev < MEDIUM_AGGREGATION_STDDEV)
  {
    leds_on(LEDS_GREEN);
  }
  else if (StdDev < HIGH_AGGREGATION_STDDEV)
  {
    leds_on(LEDS_BLUE);
  }
  else
  {
    // High segment size = buffer size, no aggregation
    leds_on(LEDS_RED);
  }
}
