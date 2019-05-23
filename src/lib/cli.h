#include <string.h>
#include <stdlib.h>

#define CR '\r'
#define LF '\n'
#define BS '\b'
#define NULLCHAR '\0'
#define SPACE ' '

#define COMMAND_BUFFER_LENGTH        25                        //length of serial buffer for incoming commands
char   CommandLine[COMMAND_BUFFER_LENGTH + 1];                 //Read commands into this buffer from Serial.  +1 in length for a termination char

const char *delimiters = ", \n";                               //commands can be separated by return, space or comma

const char *statusCommandToken = "status";
const char *rebootCommandToken = "reboot";
const char *profileCommandToken = "config";
const char *freqCommandToken = "freq";

/*************************************************************************************************************
    getCommandLineFromSerialPort()
      Return the string of the next command. Commands are delimited by return"
      Handle BackSpace character
      Make all chars lowercase
*************************************************************************************************************/

bool getCommandLineFromSerialPort(char * commandLine) {
  static uint8_t charsRead = 0;                      //note: COMAND_BUFFER_LENGTH must be less than 255 chars long
  //read asynchronously until full command input
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case CR:      //likely have full command in buffer now, commands are terminated by CR and/or LS
      case LF:
        commandLine[charsRead] = NULLCHAR;       //null terminate our command char array
        if (charsRead > 0)  {
          charsRead = 0;                           //charsRead is static, so have to reset
          Serial.println(commandLine);
          return true;
        }
        break;
      case BS:                                    // handle backspace in input: put a space in last char
        if (charsRead > 0) {                        //and adjust commandLine and charsRead
          commandLine[--charsRead] = NULLCHAR;
          Serial << byte(BS) << byte(SPACE) << byte(BS);  //no idea how this works, found it on the Internet
        }
        break;
      default:
        // c = tolower(c);
        if (charsRead < COMMAND_BUFFER_LENGTH) {
          commandLine[charsRead++] = c;
        }
        commandLine[charsRead] = NULLCHAR;     //just in case
        break;
    }
  }
  return false;
}


/* ****************************
   readNumber: return a 16bit (for Arduino Uno) signed integer from the command line
   readWord: get a text word from the command line

*/
int readNumber () {
  char * numTextPtr = strtok(NULL, delimiters);         //K&R string.h  pg. 250
  return atoi(numTextPtr);                              //K&R string.h  pg. 251
}

char * readWord() {
  char * word = strtok(NULL, delimiters);               //K&R string.h  pg. 250
  return word;
}

void nullCommand(char * ptrToCommandName) {
  Serial.println("Command not found: "+String(ptrToCommandName));      //see above for macro print2
}

// ----------------------------------------------------------------------------- commands

// will be called before and after flashing and to verify after set of freq and profile
int statusCommand() {
  Serial.print(VERSION);
  Serial.print("###");
  Serial.print("433"); // freq
  Serial.print("###");
  Serial.print("2"); // profile number
  return 0;
}

// will be called after freq and profile is set
int rebootCommand() {
  delay(1000);
  ESP.restart();
  return 0;
}

int profileCommand() {
  int pnum = -1;
  pnum = readNumber();
  if (pnum >= 0 && pnum <= 3) {
    // cfg.profile = pnum;
    return 0;
  }
  else return 1;
}
int freqCommand() {
  int freq = -1;
  freq = readNumber();
  if (freq == 433 || freq == 868 || freq == 915) {
    // cfg.freq = freq * 1000000;
    return 0;
  } else return 1;
}
// ----------------------------------------------------------------------------- run command
bool runCommand(char * commandLine) {
  int result;
  char * ptrToCommandName = strtok(commandLine, delimiters);

  if (strcmp(ptrToCommandName, statusCommandToken) == 0) {
    statusCommand();
  } else if (strcmp(ptrToCommandName, rebootCommandToken) == 0) {
    rebootCommand();                                       //K&R string.h  pg. 251
  } else if (strcmp(ptrToCommandName, profileCommandToken) == 0) {
    result = profileCommand();                                       //K&R string.h  pg. 251
    if (result == 0) Serial.println("ok");
    else Serial.println("failed");
  } else if (strcmp(ptrToCommandName, freqCommandToken) == 0) {
    result = freqCommand();                                       //K&R string.h  pg. 251
    if (result == 0) Serial.println("ok");
    else Serial.println("failed");
  } else {
      nullCommand(ptrToCommandName);
  }

}
