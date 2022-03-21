#include "DHT.h"
#include <SimpleCLI.h>

/* Definitions and global variables */

/* Pins */
#define DHTPIN    A1

/* Sensor types */
#define DHTTYPE   DHT11 

/* Global variables */
DHT dht(DHTPIN, DHTTYPE);

SimpleCLI cli;

/* Adding commands */
Command ReadTemperatureCommand;

/* User functions */
float get_temperature_reading()
{
  float temperature;
  
  temperature = dht.readTemperature();

  if (isnan(temperature)) {
    temperature = -1.0f;
    Serial.println("Failed to read from DHT11 sensor!");
  }
  
  return temperature;
}

/* Command functions */
void command_read_temperature() 
{
  float f = get_temperature_reading();
  Serial.print("Current temperature = ");
  Serial.print(f);
  Serial.println(" *C");
}

void setup() 
{
  Serial.begin(9600);
  
  /* Inialization */
  dht.begin();

  /* Commands initialization */
  ReadTemperatureCommand = cli.addCommand("read_temperature");
}

void loop() 
{
  if (Serial.available()) {
    String serialInput = Serial.readStringUntil('\n');
    Serial.print(F("# "));
    Serial.println(serialInput);

    cli.parse(serialInput);
  }
  
  // First check if a newly parsed command is available
  if(cli.available()) {
    // Get the command out of the queue
    Command cmd = cli.getCommand();

    // Check if it's the command you're looking for
    if (cmd == ReadTemperatureCommand) {
      command_read_temperature();
    }
  }
}
