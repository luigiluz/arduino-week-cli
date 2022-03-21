#include <SimpleCLI.h>
#include "DHT.h"

/* Definitions and global variables */

/* Pins */
#define DHTPIN    A1
#define COOLER_PIN 6

/* Sensor types */
#define DHTTYPE   DHT11 

/* Global variables */
DHT dht(DHTPIN, DHTTYPE);

SimpleCLI cli;

/* Adding commands */
Command ReadTemperatureCommand;
Command ControlCoolerCommand;
Command ControlBasedOnTemperatureCommand;

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

void activate_cooler(int pwm_val) {
  if (pwm_val < 0) {
    pwm_val = 0;
  } else if (pwm_val > 255) {
    pwm_val = 255;
  }
  analogWrite(COOLER_PIN, pwm_val);
}

void control_cooler_based_on_temperature(int vel, float temperature_threshold)
{
  float current_temperature;

  current_temperature = get_temperature_reading();

  if (current_temperature > temperature_threshold) {
    activate_cooler(vel);
  } else {
    activate_cooler(0);
  }

  delay(100);

  Serial.print(current_temperature);
  Serial.print(", ");
  Serial.println(temperature_threshold);
}

/* Command functions */
void command_read_temperature() 
{
  float f = get_temperature_reading();
  Serial.print("Current temperature = ");
  Serial.print(f);
  Serial.println(" *C");
}

void command_control_cooler(unsigned long int time_ms, int velocity)
{
  Serial.println("entrei na função");
  unsigned long previous_time = millis();
  unsigned long current_time = millis();
  unsigned long time_difference = current_time - previous_time;

  Serial.println("Tempo iniciado");
  while(time_difference < time_ms) {
    activate_cooler(velocity);
    current_time = millis();
    time_difference = current_time - previous_time;
  }
  Serial.println("Tempo finalizado");

  activate_cooler(0);
}

void command_control_based_on_temperature(int velocity, float temperature_threshold)
{
  uint8_t readChar[2]; // TODO: Mover o valor para um define

  Serial.println(F("pressione 'p' para parar"));
  Serial.println("current temp, threshold");
  while (true) {
    if (Serial.available() > 0) {
      Serial.readBytesUntil('\n', readChar, 2);
      if (readChar[0] == 'p') break;
    }
    control_cooler_based_on_temperature(velocity, temperature_threshold);
  }
}
 
void setup() 
{
  Serial.begin(9600);
  
  /* Inialization */
  dht.begin();
  pinMode(COOLER_PIN, OUTPUT);

  /* Commands initialization */
  ReadTemperatureCommand = cli.addCommand("read_temperature");

  ControlCoolerCommand = cli.addCommand("control_cooler");
  ControlCoolerCommand.addArgument("time");
  ControlCoolerCommand.addArgument("vel");

  ControlBasedOnTemperatureCommand = cli.addCommand("control_based_on_temperature");
  ControlBasedOnTemperatureCommand.addArgument("temp");
  ControlBasedOnTemperatureCommand.addArgument("vel");
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
    } else if (cmd == ControlCoolerCommand) {
      Argument timeArgument = cmd.getArgument("time");
      Argument velArgument = cmd.getArgument("vel");

      String timeArg = timeArgument.getValue();
      String velArg = velArgument.getValue();

      unsigned long int time_ms = timeArg.toInt();
      int velocity = velArg.toInt();
    
      command_control_cooler(time_ms, velocity);  
    } else if (cmd == ControlBasedOnTemperatureCommand) {
      Argument velArgument = cmd.getArgument("vel");
      Argument tempArgument = cmd.getArgument("temp");

      String velArg = velArgument.getValue();
      String tempArg = tempArgument.getValue();

      int velocity = velArg.toInt();
      float temperature_threshold = tempArg.toFloat();

      command_control_based_on_temperature(velocity, temperature_threshold);
    }
  }
}
