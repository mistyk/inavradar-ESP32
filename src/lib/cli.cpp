#include <lib/cli.h>
#include <lib/MSP.h>
#include <main.h>

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
void CLI::begin(Stream & stream)
{
  _stream   = &stream;

}

bool CLI::process() {
  static uint8_t charsRead = 0;                      //note: COMAND_BUFFER_LENGTH must be less than 255 chars long
  //read asynchronously until full command input
  while (_stream->available()) {
    char c = _stream->read();
    switch (c) {
      case CR:      //likely have full command in buffer now, commands are terminated by CR and/or LS
      case LF:
        commandLine[charsRead] = NULLCHAR;       //null terminate our command char array
        if (charsRead > 0)  {
          charsRead = 0;                           //charsRead is static, so have to reset
          //_stream->println(commandLine);
          runCommand(commandLine);
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

void CLI::nullCommand(char * ptrToCommandName) {
  _stream->println("Command not found: "+String(ptrToCommandName));      //see above for macro print2
}

// ----------------------------------------------------------------------------- commands

// will be called before and after flashing and to verify after set of freq and profile
int CLI::statusCommand() {
  _stream->println("Status");
  _stream->print("Version: ");
  _stream->println(VERSION);
  _stream->print("Name: ");
  _stream->println(String(curr.name)); // freq
  _stream->print("Profile: ");
  _stream->println(cfg.profile_id); // profile number
  _stream->print("PPS: ");
  _stream->println(sys.pps);
  _stream->print("Peers: ");
  _stream->println(sys.num_peers_active);
  for (size_t i = 0; i < cfg.lora_nodes_max; i++) {
    _stream->println("---");
    _stream->print("id: ");
    _stream->println(peers[i].id);
    _stream->print("host: ");
    _stream->println(peers[i].host);
    _stream->print("state: ");
    _stream->println(peers[i].state);
    _stream->print("lost: ");
    _stream->println(peers[i].lost);
    _stream->print("updated: ");
    _stream->println(peers[i].updated);
    _stream->print("lq: ");
    _stream->println(peers[i].lq);
    _stream->print("rssi: ");
    _stream->println(peers[i].rssi);
    _stream->print("lat: ");
    _stream->println(String((float)peers[i].gpsrec.lat / 10000000, 6));
    _stream->print("lon: ");
    _stream->println(String((float)peers[i].gpsrec.lon / 10000000, 6));
    _stream->print("alt: ");
    _stream->println(peers[i].gpsrec.alt);
    _stream->print("speed: ");
    _stream->println(peers[i].gpsrec.groundSpeed / 100);
    _stream->print("course: ");
    _stream->println(peers[i].gpsrec.groundCourse / 10);
  }

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
bool CLI::runCommand(char * commandLine) {
  int result;
  char * ptrToCommandName = strtok(commandLine, delimiters);

  if (strcmp(ptrToCommandName, statusCommandToken) == 0) {
    statusCommand();
    if (result == 0) _stream->println("ok");
    else _stream->println("failed");
  } else if (strcmp(ptrToCommandName, rebootCommandToken) == 0) {
    rebootCommand();                                       //K&R string.h  pg. 251
  } else if (strcmp(ptrToCommandName, profileCommandToken) == 0) {
    result = profileCommand();                                       //K&R string.h  pg. 251
    if (result == 0) _stream->println("ok");
    else _stream->println("failed");
  } else if (strcmp(ptrToCommandName, freqCommandToken) == 0) {
    result = freqCommand();                                       //K&R string.h  pg. 251
    if (result == 0) _stream->println("ok");
    else _stream->println("failed");
  } else {
      nullCommand(ptrToCommandName);
  }

}
