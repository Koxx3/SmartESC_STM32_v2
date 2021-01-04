/*
 * TTerm
 *
 * Copyright (c) 2020 Thorben Zethoff
 * Copyright (c) 2020 Jens Kerrinnes
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#if !defined(TTerm_H)
#define TTerm_H
    
#ifndef _VT100_CURSOR_POS1

#include "FreeRTOS.h"
#include "task.h"
  
#if PIC32 == 1    
#include "ff.h"
#endif    

#define EXTENDED_PRINTF 0
#define TERM_DEVICE_NAME "m365"
#define TERM_VERSION_STRING "V1.0"

#if PIC32 == 1 
    #define START_OF_FLASH  0xa0000000
    #define END_OF_FLASH    0xa000ffff
#else
    #define START_OF_FLASH  0x00000000
    #define END_OF_FLASH    0x1FFF8000
#endif

enum vt100{
    _VT100_CURSOR_POS1,
    _VT100_CURSOR_END,
    _VT100_FOREGROUND_COLOR,
    _VT100_BACKGROUND_COLOR,
    _VT100_RESET_ATTRIB,
    _VT100_BRIGHT,
    _VT100_DIM,
    _VT100_UNDERSCORE,
    _VT100_BLINK,
    _VT100_REVERSE,
    _VT100_HIDDEN,
    _VT100_ERASE_SCREEN,
    _VT100_ERASE_LINE,
    _VT100_FONT_G0,
    _VT100_FONT_G1,
    _VT100_WRAP_ON,
    _VT100_WRAP_OFF,
    _VT100_ERASE_LINE_END,
    _VT100_CURSOR_BACK_BY,
    _VT100_CURSOR_FORWARD_BY,
    _VT100_CURSOR_DOWN_BY,
    _VT100_CURSOR_UP_BY,
    _VT100_CURSOR_SAVE_POSITION,
    _VT100_CURSOR_RESTORE_POSITION,
    _VT100_CURSOR_ENABLE,
    _VT100_CURSOR_DISABLE,
    _VT100_CURSOR_SET_COLUMN,
    _VT100_CLS
};

//VT100 cmds given to us by the terminal software (they need to be > 8 bits so the handler can tell them apart from normal characters)
#define _VT100_RESET                0x1000
#define _VT100_KEY_END              0x1001
#define _VT100_KEY_POS1             0x1002
#define _VT100_CURSOR_FORWARD       0x1003
#define _VT100_CURSOR_BACK          0x1004
#define _VT100_CURSOR_UP            0x1005
#define _VT100_CURSOR_DOWN          0x1006
#define _VT100_BACKWARDS_TAB        0x1007

enum color{
    _VT100_BLACK,
    _VT100_RED,
    _VT100_GREEN,
    _VT100_YELLOW,
    _VT100_BLUE,
    _VT100_MAGENTA,
    _VT100_CYAN,
    _VT100_WHITE
};

#define TERM_addCommandConstAC(CMDhandler, command, helptext, ACList, CmdDesc) TERM_addCommandAC(TERM_addCommand(CMDhandler, command,helptext,0,CmdDesc) \
                                                                                , ACL_defaultCompleter, ACL_createConst(ACList, sizeof(ACList)/sizeof(char*)))


#define _VT100_POS_IGNORE 0xffff

#define TERM_HISTORYSIZE 16
#define TERM_INPUTBUFFER_SIZE 128

                        
#define TERM_ARGS_ERROR_STRING_LITERAL 0xffff

#define TERM_CMD_EXIT_ERROR 0
#define TERM_CMD_EXIT_NOT_FOUND 1
#define TERM_CMD_EXIT_SUCCESS 0xff
#define TERM_CMD_EXIT_PROC_STARTED 0xfe
#define TERM_CMD_CONTINUE 0x80

#define TERM_ENABLE_STARTUP_TEXT
//#define TERM_SUPPORT_CWD

#ifdef TERM_ENABLE_STARTUP_TEXT
const extern char TERM_startupText1[];
const extern char TERM_startupText2[];
const extern char TERM_startupText3[];
#endif

#define ttprintfEcho(format, ...) if(handle->echoEnabled) (*handle->print)(format, ##__VA_ARGS__)


#define ttprintf(format, ...) (*handle->print)(format, ##__VA_ARGS__)
#define ttprintb(buffer, len) (*handle->print)(NULL, (uint8_t*)buffer, (uint32_t)len)


typedef struct __TERMINAL_HANDLE__ TERMINAL_HANDLE;
typedef struct __TermCommandDescriptor__ TermCommandDescriptor;

typedef uint8_t (* TermCommandFunction)(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
typedef uint8_t (* TermCommandInputHandler)(TERMINAL_HANDLE * handle, uint16_t c);
typedef uint8_t (* TermErrorPrinter)(TERMINAL_HANDLE * handle, uint32_t retCode);


typedef int (* TermPrintHandler)(const char * format, ...);

typedef uint8_t (* TermAutoCompHandler)(TERMINAL_HANDLE * handle, void * params);

typedef struct{
    TaskHandle_t task;
    TermCommandInputHandler inputHandler;
} TermProgram;

struct __TermCommandDescriptor__{
    TermCommandFunction function;
    const char * command;
    const char * commandDescription;
    uint32_t commandLength;
    uint8_t minPermissionLevel;
    TermAutoCompHandler ACHandler;
    void * ACParams;
    
    TermCommandDescriptor * nextCmd;
};

struct __TERMINAL_HANDLE__{
    char * inputBuffer;
    uint32_t currBufferPosition;
    uint32_t currBufferLength;
    uint32_t currAutocompleteCount;
    TermProgram * currProgram;
    char ** autocompleteBuffer;
    uint32_t autocompleteBufferLength;
    uint32_t autocompleteStart;    
    TermPrintHandler print;
    char * currUserName;
    char * historyBuffer[TERM_HISTORYSIZE];
    uint32_t currHistoryWritePosition;
    uint32_t currHistoryReadPosition;
    uint8_t currEscSeqPos;
    uint8_t escSeqBuff[16];
    unsigned echoEnabled;
    TermCommandDescriptor * cmdListHead;
    TermErrorPrinter errorPrinter;
//TODO actually finish implementing this...
#ifdef TERM_SUPPORT_CWD
    DIR cwd;
#endif
};

typedef enum{
    TERM_CHECK_COMP_AND_HIST = 0b11, TERM_CHECK_COMP = 0b01, TERM_CHECK_HIST = 0b10, 
} COPYCHECK_MODE;

extern TermCommandDescriptor TERM_cmdListHead;


TERMINAL_HANDLE * TERM_createNewHandle(TermPrintHandler printFunction, unsigned echoEnabled, TermCommandDescriptor * cmdListHead, TermErrorPrinter errorPrinter, const char * usr);    

void TERM_destroyHandle(TERMINAL_HANDLE * handle);
uint8_t TERM_processBuffer(uint8_t * data, uint16_t length, TERMINAL_HANDLE * handle);
unsigned isACIILetter(char c);
uint8_t TERM_handleInput(uint16_t c, TERMINAL_HANDLE * handle);
char * strnchr(char * str, char c, uint32_t length);
void strsft(char * src, int32_t startByte, int32_t offset);
void TERM_printBootMessage(TERMINAL_HANDLE * handle);
void TERM_freeCommandList(TermCommandDescriptor ** cl, uint16_t length);
uint8_t TERM_buildCMDList();
TermCommandDescriptor * TERM_addCommand(TermCommandFunction function, const char * command, const char * description, uint8_t minPermissionLevel, TermCommandDescriptor * head);
void TERM_addCommandAC(TermCommandDescriptor * cmd, TermAutoCompHandler ACH, void * ACParams);
unsigned TERM_isSorted(TermCommandDescriptor * a, TermCommandDescriptor * b);
char toLowerCase(char c);
void TERM_setCursorPos(TERMINAL_HANDLE * handle, uint16_t x, uint16_t y);
void TERM_Box(TERMINAL_HANDLE * handle, uint8_t row1, uint8_t col1, uint8_t row2, uint8_t col2);
void TERM_sendVT100Code(TERMINAL_HANDLE * handle, uint16_t cmd, uint8_t var);
const char * TERM_getVT100Code(uint16_t cmd, uint8_t var);
uint16_t TERM_countArgs(const char * data, uint16_t dataLength);
uint8_t TERM_interpretCMD(char * data, uint16_t dataLength, TERMINAL_HANDLE * handle);
uint16_t TERM_seperateArgs(char * data, uint16_t dataLength, char ** buff);
void TERM_checkForCopy(TERMINAL_HANDLE * handle, COPYCHECK_MODE mode);
void TERM_printDebug(TERMINAL_HANDLE * handle, char * format, ...);
void TERM_removeProgramm(TERMINAL_HANDLE * handle);
void TERM_attachProgramm(TERMINAL_HANDLE * handle, TermProgram * prog);
uint8_t TERM_doAutoComplete(TERMINAL_HANDLE * handle);
uint8_t TERM_findMatchingCMDs(char * currInput, uint8_t length, char ** buff, TermCommandDescriptor * cmdListHead);
uint8_t TERM_findLastArg(TERMINAL_HANDLE * handle, char * buff, uint8_t * lenBuff);
BaseType_t ptr_is_in_ram(void* ptr);
uint8_t TERM_defaultErrorPrinter(TERMINAL_HANDLE * handle, uint32_t retCode);
void TERM_LIST_add(TermCommandDescriptor * item, TermCommandDescriptor * head);
TermCommandDescriptor * TERM_findCMD(TERMINAL_HANDLE * handle);

#endif
#endif
