#include <string.h>
#include <stdlib.h>
#include <Stream.h>

#define CR '\r'
#define LF '\n'
#define BS '\b'
#define NULLCHAR '\0'
#define SPACE ' '

#define COMMAND_BUFFER_LENGTH        25                        //length of serial buffer for incoming commands



class CLI {

  public:
    void begin(Stream & stream);
    bool runCommand(char * commandLine);
    bool process();
    char commandLine[COMMAND_BUFFER_LENGTH + 1];                 //Read commands into this buffer from serialConsole[cNum]->  +1 in length for a termination char

  private:
    Stream * _stream;
    void nullCommand(char * ptrToCommandName) ;
    int statusCommand();

};
