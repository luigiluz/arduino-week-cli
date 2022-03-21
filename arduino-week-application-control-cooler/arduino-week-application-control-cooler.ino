#include <SimpleCLI.h>

/* Definitions and global variables */
/* Pins */
#define COOLER_PIN 6

SimpleCLI cli;

/* Adding commands */
Command ControlCoolerCommand;

/* User functions */
void activate_cooler(int pwm_val) {
  if (pwm_val < 0) {
    pwm_val = 0;
  } else if (pwm_val > 255) {
    pwm_val = 255;
  }
  analogWrite(COOLER_PIN, pwm_val);
}

/* Command functions */
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
 
void setup() 
{
  Serial.begin(9600);
  
  /* Inialization */
  pinMode(COOLER_PIN, OUTPUT);

  /* Commands initialization */
  ControlCoolerCommand = cli.addCommand("control_cooler");
  ControlCoolerCommand.addArgument("time");
  ControlCoolerCommand.addArgument("vel");
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
    if (cmd == ControlCoolerCommand) {
      Argument timeArgument = cmd.getArgument("time");
      Argument velArgument = cmd.getArgument("vel");

      String timeArg = timeArgument.getValue();
      String velArg = velArgument.getValue();

      unsigned long int time_ms = timeArg.toInt();
      int velocity = velArg.toInt();
    
      command_control_cooler(time_ms, velocity);  
    }
  }
}
