// ------------------------------------------------------------------------------------------------------
//
// # DESCRIPTION : Basic CLI (Command Line Interpreter) library for Arduino.
//
//   The CLI class first allows the user to register commands (i.e. a combination of a name and a function
//   address) into a linked list.
//   Then, on every LF (Line Feed) char reception on the Serial Interface, the class checks whether the
//   input command name matches a registered one. In this case, the corresponding function is executed.
//
//   NB1 : Each command name is 8 characters MAX by default.
//   NB2 : The functions associated to the commands shall be "void function(void)" typed.
//
// # USAGE :
//   1. in the setup() Arduino sketch function, register all your commands using RegisterCmd() method
//      and open the serial port.
//   2. in the loop() Arduino sketch function, call the Run() method to allow CLI execution.
//
//   NB : If you don't use the Arduino serial monitor, make sure your terminal application is configured
//        with a 'LF' char for a New Line ('CR' char is ignored).
// ------------------------------------------------------------------------------------------------------

#ifndef CLI_H
#define CLI_H

#include "HardwareSerial.h"

#define CMD_NAME_MAX_LENGTH 8

typedef void (*t_fct)(void); // typedef for function pointer

// typedef for the linked list of command lines
typedef struct t_cmd {
  char name[CMD_NAME_MAX_LENGTH + 1];
  t_fct fct;
  struct t_cmd *next;
};


class Cli{
    HardwareSerial& _serial; // serial interface to be used
    t_cmd *_cmdList;  // linked list of commands

  public :
    // Constructor
    Cli(HardwareSerial& serial) : _serial(serial) { _cmdList = NULL;}

    // Destructor
    ~Cli()
    {
      t_cmd *current, *next;
      for (current = _cmdList; current; current = next)
      {
        next = current->next;
        delete(current);
      }
    }

    // RegisterCmd() function
    // Allow the user to register a new user command
    // Return an error (-1) when :
    //   - the passed command name is null or too long
    //   - the passed function @ is null
    //   - no new memory can be allocated
    // else return 0
    char RegisterCmd(const char name[], t_fct fct)
    {
    unsigned int length;
      // check params
      if (fct == NULL) return -1;
      for (length = 0; name[length] ; length++);
      if ((!length) || (length > CMD_NAME_MAX_LENGTH)) return -1;

      // create new command
      t_cmd *newCmd = new(t_cmd);
      if (newCmd == NULL) return -1; // allocation failure
      newCmd->fct = fct;
      for (unsigned int i=0; i<=length; i++) newCmd->name[i] = name[i];

      // attach the new command to the linked list
      if (_cmdList == NULL)
      {
        _cmdList = newCmd ; _cmdList->next = NULL;
      }
      else
      {
        newCmd->next = _cmdList;
        _cmdList = newCmd;
      }
      return 0;
    }

    // Run() function
    // To be called in loop() Arduino sketch function
    // Read the serial interface, and execute a given function when the input text matches
    void Run(void)
    {
    static char inData[CMD_NAME_MAX_LENGTH + 1];
    char newChar;
    static unsigned int index = 0;
    t_cmd *current;
    unsigned char i;

      while (_serial.available() > 0)
      {
        newChar = _serial.read();
        if (newChar == '\r') continue; // ignore CR chars
        if (newChar == '\n')
        {
          inData[index] = NULL;
          index = 0;
          for (current = _cmdList; current != NULL ; current = current->next)
          {
            for(i=0; (inData[i] == current->name[i]) && current->name[i]; i++);
            if ((!inData[i]) && (!current->name[i]))
            {
              current->fct();
              break; // execute one command only
            }
          }
        }
        else if (index == CMD_NAME_MAX_LENGTH) index = 0; // restart acquisition from zero
        else inData[index++] = newChar;
      }
    }
};


#endif // CLI_H
