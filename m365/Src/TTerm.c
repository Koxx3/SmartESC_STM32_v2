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
    
#if PIC32 == 1
#include <xc.h>
#endif    
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "FreeRTOS.h"
#include "task.h"
#//include "UART.h"
#include "TTerm.h"
#include "TTerm_cmd.h"
#include "TTerm_AC.h"
//#include "cli_common.h"


#if TTERM_ENABLED

TermCommandDescriptor TERM_cmdListHead = {.nextCmd = 0, .commandLength = 0};
unsigned TERM_baseCMDsAdded = 0;

#define ESC_STR "\x1b"


TERMINAL_HANDLE * TERM_createNewHandle(TermPrintHandler printFunction, unsigned echoEnabled, TermCommandDescriptor * cmdListHead, const char * usr){

    TERMINAL_HANDLE * handle = pvPortMalloc(sizeof(TERMINAL_HANDLE));
    memset(handle, 0, sizeof(TERMINAL_HANDLE));
    handle->inputBuffer = pvPortMalloc(TERM_INPUTBUFFER_SIZE);
    handle->print = printFunction;
    handle->cmdListHead = cmdListHead;
    handle->currUserName = pvPortMalloc(strlen(usr) + 1 + strlen(TERM_getVT100Code(_VT100_FOREGROUND_COLOR, _VT100_YELLOW)) + strlen(TERM_getVT100Code(_VT100_RESET_ATTRIB, 0)));
    sprintf(handle->currUserName, "%s%s%s", TERM_getVT100Code(_VT100_FOREGROUND_COLOR, _VT100_BLUE), usr, TERM_getVT100Code(_VT100_RESET_ATTRIB, 0));
    handle->currEscSeqPos = 0xff;
    

    handle->errorPrinter = TERM_defaultErrorPrinter;



    //if this is the first console we initialize we need to add the static commands
    if(!TERM_baseCMDsAdded){
        TERM_baseCMDsAdded = 1;
        
        TERM_addCommand(CMD_help, "help", "Displays this help message", 0, &TERM_cmdListHead);
        TERM_addCommand(CMD_cls, "cls", "Clears the screen", 0, &TERM_cmdListHead);
        TERM_addCommand(CMD_top, "top", "shows performance stats", 0, &TERM_cmdListHead);

    }
    
#ifdef TERM_ENABLE_STARTUP_TEXT
    TERM_printBootMessage(handle);
#endif
    ttprintf("\r\n\r\n%s@%s>", handle->currUserName, TERM_DEVICE_NAME);
    return handle;
}

void TERM_destroyHandle(TERMINAL_HANDLE * handle){
    if(handle->currProgram != NULL){
        //try to gracefully shutdown the running program
        (*handle->currProgram->inputHandler)(handle, 0x03);
    }
    
    vPortFree(handle->inputBuffer);
    vPortFree(handle->currUserName);
    
    uint8_t currHistoryPos = 0;
    for(;currHistoryPos < TERM_HISTORYSIZE; currHistoryPos++){
        if(handle->historyBuffer[currHistoryPos] != 0){
            vPortFree(handle->historyBuffer[currHistoryPos]);
            handle->historyBuffer[currHistoryPos] = 0;
        }
    }
    
    vPortFree(handle->autocompleteBuffer);
    handle->autocompleteBuffer = NULL;
    vPortFree(handle);
}

void TERM_printDebug(TERMINAL_HANDLE * handle, char * format, ...){
    //TODO make this nicer... we don't need a double buffer allocation, we should instead send the va_list to the print function. But it is way to late at night for me to code this now...
    //TODO implement a debug level control in the terminal handle (permission level?)
    va_list arg;
    va_start(arg, format);

    char * buff = (char*) pvPortMalloc(256);
    vsnprintf(buff, 256,format, arg);

    ttprintf("\r\n%s", buff);

    if(handle->currBufferLength == 0){
        ttprintf("%s@%s>", handle->currUserName, TERM_DEVICE_NAME);
    }else{
        ttprintf("%s@%s>%s", handle->currUserName, TERM_DEVICE_NAME, handle->inputBuffer);
        if(handle->inputBuffer[handle->currBufferPosition] != 0) TERM_sendVT100Code(handle, _VT100_CURSOR_BACK_BY, handle->currBufferLength - handle->currBufferPosition);
    }

    vPortFree(buff);

    va_end(arg);
}

uint8_t TERM_processBuffer(uint8_t * data, uint16_t length, TERMINAL_HANDLE * handle){
    uint16_t currPos = 0;
    for(;currPos < length; currPos++){
        //ttprintfEcho("checking 0x%02x\r\n", data[currPos]);
        if(handle->currEscSeqPos != 0xff){
            if(handle->currEscSeqPos == 0){
                if(data[currPos] == '['){
                    handle->escSeqBuff[handle->currEscSeqPos++] = data[currPos];
                }else{
                    switch(data[currPos]){
                        case 'c':
                            handle->currEscSeqPos = 0xff;
                            TERM_handleInput(_VT100_RESET, handle);
                            break;
                        case 0x1b:
                            handle->currEscSeqPos = 0;
                            break;
                        default:
                            handle->currEscSeqPos = 0xff;
                            TERM_handleInput(0x1b, handle);
                            TERM_handleInput(data[currPos], handle);
                            break;
                    }
                }
            }else{
                if(isACIILetter(data[currPos])){
                    if(data[currPos] == 'n'){
                        if(handle->currEscSeqPos == 2){
                            if(handle->escSeqBuff[0] == '5'){        //Query device status
                            }else if(handle->escSeqBuff[0] == '6'){  //Query cursor position
                            }
                        }
                    }else if(data[currPos] == 'c'){
                        if(handle->currEscSeqPos == 1){              //Query device code
                        }
                    }else if(data[currPos] == 'F'){
                        if(handle->currEscSeqPos == 1){              //end
                            TERM_handleInput(_VT100_KEY_END, handle);
                        }
                    }else if(data[currPos] == 'H'){
                        if(handle->currEscSeqPos == 1){              //pos1
                            TERM_handleInput(_VT100_KEY_POS1, handle);
                        }
                    }else if(data[currPos] == 'C'){                      //cursor forward
                        if(handle->currEscSeqPos > 1){              
                            handle->escSeqBuff[handle->currEscSeqPos] = 0;
                        }else{
                            TERM_handleInput(_VT100_CURSOR_FORWARD, handle);
                        }
                    }else if(data[currPos] == 'D'){                      //cursor backward
                        if(handle->currEscSeqPos > 1){                 
                            handle->escSeqBuff[handle->currEscSeqPos] = 0;
                        }else{
                            TERM_handleInput(_VT100_CURSOR_BACK, handle);
                        }
                    }else if(data[currPos] == 'A'){                      //cursor up
                        if(handle->currEscSeqPos > 1){                 
                            handle->escSeqBuff[handle->currEscSeqPos] = 0;
                        }else{
                            TERM_handleInput(_VT100_CURSOR_UP, handle);
                        }
                    }else if(data[currPos] == 'B'){                      //cursor down
                        if(handle->currEscSeqPos > 1){                 
                            handle->escSeqBuff[handle->currEscSeqPos] = 0;
                        }else{
                            TERM_handleInput(_VT100_CURSOR_DOWN, handle);
                        }
                    }else if(data[currPos] == 'Z'){                      //shift tab or ident request(at least from exp.; didn't find any official spec containing this)
                        TERM_handleInput(_VT100_BACKWARDS_TAB, handle);
                    }else{                      //others
                        handle->escSeqBuff[handle->currEscSeqPos+1] = 0;
                    }
                    handle->currEscSeqPos = 0xff;
                }else{
                    handle->escSeqBuff[handle->currEscSeqPos++] = data[currPos];
                }
            }
        }else{
            if(data[currPos] == 0x1B){     //ESC for V100 control sequences
                handle->currEscSeqPos = 0;
            }else{
                TERM_handleInput(data[currPos], handle);
            }
        }
    }
    return TERM_CMD_EXIT_SUCCESS;
}

unsigned isACIILetter(char c){
    return (c > 64 && c < 91) || (c > 96 && c < 122);
}

void TERM_printBootMessage(TERMINAL_HANDLE * handle){
    TERM_sendVT100Code(handle, _VT100_RESET, 0); TERM_sendVT100Code(handle, _VT100_CURSOR_POS1, 0); TERM_sendVT100Code(handle, _VT100_WRAP_OFF, 0);
    ttprintf("\r\n\n\n%s\r\n", TERM_startupText1);
    ttprintf("%s\r\n", TERM_startupText2);
    ttprintf("%s\r\n", TERM_startupText3);
}

BaseType_t ptr_is_in_ram(void* ptr){
    if(ptr > (void*)START_OF_FLASH && ptr < (void*)END_OF_FLASH){
        return pdFALSE;
    }
    return pdTRUE;
}

uint8_t TERM_defaultErrorPrinter(TERMINAL_HANDLE * handle, uint32_t retCode){
    switch(retCode){
        case TERM_CMD_EXIT_SUCCESS:
            ttprintf("\r\n%s@%s>", handle->currUserName, TERM_DEVICE_NAME);
            break;

        case TERM_CMD_EXIT_ERROR:
            ttprintf("\r\nTask returned with error code %d\r\n%s@%s>", retCode, handle->currUserName, TERM_DEVICE_NAME);
            break;

        case TERM_CMD_EXIT_NOT_FOUND:
            ttprintf("\"%s\" is not a valid command. Type \"help\" to see a list of available ones\r\n%s@%s>", handle->inputBuffer, handle->currUserName, TERM_DEVICE_NAME);
            break;
    }
    return TERM_CMD_EXIT_SUCCESS;
}

uint8_t TERM_handleInput(uint16_t c, TERMINAL_HANDLE * handle){
    if(handle->currProgram != NULL){
        //call the handler of the current override
        //uint8_t currRetCode = (*handle->currProgram->inputHandler)(handle, c);
        
        (*handle->errorPrinter)(handle, TERM_CMD_EXIT_NOT_FOUND);
        
        if(c == 0x03){
            ttprintf("^C");
        }
        return 1;
    }
    
    switch(c){
        case '\r':      //enter
            TERM_checkForCopy(handle, TERM_CHECK_COMP_AND_HIST);
            
            if(handle->currBufferLength != 0){
                ttprintf("\r\n", handle->inputBuffer);

                if(handle->historyBuffer[handle->currHistoryWritePosition] != 0){
                    vPortFree(handle->historyBuffer[handle->currHistoryWritePosition]);
                    handle->historyBuffer[handle->currHistoryWritePosition] = 0;
                }

                handle->historyBuffer[handle->currHistoryWritePosition] = pvPortMalloc(handle->currBufferLength + 1);
                memcpy(handle->historyBuffer[handle->currHistoryWritePosition], handle->inputBuffer, handle->currBufferLength + 1);
                
                if(++handle->currHistoryWritePosition >= TERM_HISTORYSIZE) handle->currHistoryWritePosition = 0;
                
                handle->currHistoryReadPosition = handle->currHistoryWritePosition;

                uint8_t retCode = TERM_interpretCMD(handle->inputBuffer, handle->currBufferLength, handle);
                (*handle->errorPrinter)(handle, retCode);
                
                handle->currBufferPosition = 0;
                handle->currBufferLength = 0;
                handle->inputBuffer[handle->currBufferPosition] = 0;
            }else{
                ttprintf("\r\n%s@%s>", handle->currUserName, TERM_DEVICE_NAME);
            }       
            break;
            
        case 0x03:      //CTRL+c
            //TODO reset current buffer
            ttprintf("^C");
            break;
            
        case 0x08:      //backspace (used by xTerm)
        case 0x7f:      //DEL       (used by hTerm)
            TERM_checkForCopy(handle, TERM_CHECK_COMP_AND_HIST);
            if(handle->currBufferPosition == 0) break;
            
            if(handle->inputBuffer[handle->currBufferPosition] != 0){      //check if we are at the end of our command
                //we are somewhere in the middle -> move back existing characters
                strsft(handle->inputBuffer, handle->currBufferPosition - 1, -1);    
                ttprintf("\x08");
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE_END, 0);
                ttprintf("%s", &handle->inputBuffer[handle->currBufferPosition - 1]);
                TERM_sendVT100Code(handle, _VT100_CURSOR_BACK_BY, handle->currBufferLength - handle->currBufferPosition);
                handle->currBufferPosition --;
                handle->currBufferLength --;
            }else{
                //we are somewhere at the end -> just delete the current one
                handle->inputBuffer[--handle->currBufferPosition] = 0;
                ttprintf("\x08 \x08");
                handle->currBufferLength --;
            }
            break;
            
        case _VT100_KEY_END:
            TERM_checkForCopy(handle, TERM_CHECK_COMP_AND_HIST);
            //TODO move cursor to EOL
            break;
            
        case _VT100_KEY_POS1:
            TERM_checkForCopy(handle, TERM_CHECK_COMP_AND_HIST);
            //TODO move cursor to BOL
            break;
            
        case _VT100_CURSOR_FORWARD:
            TERM_checkForCopy(handle, TERM_CHECK_COMP_AND_HIST);
            
            if(handle->currBufferPosition < handle->currBufferLength){
                handle->currBufferPosition ++;
                TERM_sendVT100Code(handle, _VT100_CURSOR_FORWARD, 0);
            }
            break;
            
        case _VT100_CURSOR_BACK:
            TERM_checkForCopy(handle, TERM_CHECK_COMP_AND_HIST);
            
            if(handle->currBufferPosition > 0){
                handle->currBufferPosition --;
                TERM_sendVT100Code(handle, _VT100_CURSOR_BACK, 0);
            }
            break;
            
        case _VT100_CURSOR_UP:
            TERM_checkForCopy(handle, TERM_CHECK_COMP);
            
            do{
                if(--handle->currHistoryReadPosition >= TERM_HISTORYSIZE) handle->currHistoryReadPosition = TERM_HISTORYSIZE - 1;
                
                if(handle->historyBuffer[handle->currHistoryReadPosition] != 0){
                    break;
                }
            }while(handle->currHistoryReadPosition != handle->currHistoryWritePosition);
            
            //print out the command at the current history read position
            if(handle->currHistoryReadPosition == handle->currHistoryWritePosition){
                ttprintf("\x07");   //rings a bell doesn't it?
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE, 0);
                ttprintf("\r%s@%s>%s", handle->currUserName, TERM_DEVICE_NAME, handle->inputBuffer);
            }else{
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE, 0);
                ttprintf("\r%s@%s>%s", handle->currUserName, TERM_DEVICE_NAME, handle->historyBuffer[handle->currHistoryReadPosition]);
            }
            break;
            
        case _VT100_CURSOR_DOWN:
            TERM_checkForCopy(handle, TERM_CHECK_COMP);
            
            while(handle->currHistoryReadPosition != handle->currHistoryWritePosition){
                if(++handle->currHistoryReadPosition >= TERM_HISTORYSIZE) handle->currHistoryReadPosition = 0;
                
                if(handle->historyBuffer[handle->currHistoryReadPosition] != 0){
                    break;
                }
            }
            
            //print out the command at the current history read position
            if(handle->currHistoryReadPosition == handle->currHistoryWritePosition){
                ttprintf("\x07");   //rings a bell doesn't it?
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE, 0);
                ttprintf("\r%s@%s>%s", handle->currUserName, TERM_DEVICE_NAME, handle->inputBuffer);
            }else{
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE, 0);
                ttprintf("\r%s@%s>%s", handle->currUserName, TERM_DEVICE_NAME, handle->historyBuffer[handle->currHistoryReadPosition]);
            }
            break;
            
        case '\t':      //tab
            TERM_checkForCopy(handle, TERM_CHECK_HIST);
            
            if(handle->autocompleteBuffer == NULL){ 
                TERM_doAutoComplete(handle);
            }
            
            if(++handle->currAutocompleteCount > handle->autocompleteBufferLength) handle->currAutocompleteCount = 0;
            
            if(handle->currAutocompleteCount == 0){
                ttprintf("\x07");
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE, 0);
                ttprintf("\r%s@%s>%s", handle->currUserName, TERM_DEVICE_NAME, handle->inputBuffer);
            }else{
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE, 0);
                unsigned printQuotationMarks = strchr(handle->autocompleteBuffer[handle->currAutocompleteCount - 1], ' ') != 0;
                ttprintf(printQuotationMarks ? "\r%s@%s>%.*s\"%s\"" : "\r%s@%s>%.*s%s", handle->currUserName, TERM_DEVICE_NAME, handle->autocompleteStart, handle->inputBuffer, handle->autocompleteBuffer[handle->currAutocompleteCount - 1]);
            }
            break;
            
        case _VT100_RESET:
            TERM_printBootMessage(handle);
            break;
           
        case 32 ... 126:
            TERM_checkForCopy(handle, TERM_CHECK_COMP_AND_HIST);
            
            //TODO check for string length overflow
            
            if(handle->inputBuffer[handle->currBufferPosition] != 0){      //check if we are at the end of our command
                strsft(handle->inputBuffer, handle->currBufferPosition, 1);   
                handle->inputBuffer[handle->currBufferPosition] = c; 
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE_END, 0);
                ttprintf("%s", &handle->inputBuffer[handle->currBufferPosition]);
                TERM_sendVT100Code(handle, _VT100_CURSOR_BACK_BY, handle->currBufferLength - handle->currBufferPosition);
                handle->currBufferLength ++;
                handle->currBufferPosition ++;
            }else{
                
                //we are at the end -> just delete the current one
                handle->inputBuffer[handle->currBufferPosition++] = c;
                handle->inputBuffer[handle->currBufferPosition] = 0;
                handle->currBufferLength ++;
                ttprintf("%c", c);
            }
            break;
            
        default:
            TERM_printDebug(handle, "unknown code received: 0x%02x\r\n", c);
            break;
    }
    return TERM_CMD_EXIT_SUCCESS;
}

void TERM_checkForCopy(TERMINAL_HANDLE * handle, COPYCHECK_MODE mode){
    if((mode & TERM_CHECK_COMP) && handle->autocompleteBuffer != NULL){ 
        if(handle->currAutocompleteCount != 0){
            char * dst = (char *) ((uint32_t) handle->inputBuffer + handle->autocompleteStart);
            if(strchr(handle->autocompleteBuffer[handle->currAutocompleteCount - 1], ' ') != 0){
                sprintf(dst, "\"%s\"", handle->autocompleteBuffer[handle->currAutocompleteCount - 1]);
            }else{
                strcpy(dst, handle->autocompleteBuffer[handle->currAutocompleteCount - 1]);
            }
            handle->currBufferLength = strlen(handle->inputBuffer);
            handle->currBufferPosition = handle->currBufferLength;
            handle->inputBuffer[handle->currBufferPosition] = 0;
        }
        vPortFree(handle->autocompleteBuffer);
        handle->autocompleteBuffer = NULL;
    }
    
    if((mode & TERM_CHECK_HIST) && handle->currHistoryWritePosition != handle->currHistoryReadPosition){
        strcpy(handle->inputBuffer, handle->historyBuffer[handle->currHistoryReadPosition]);
        handle->currBufferLength = strlen(handle->inputBuffer);
        handle->currBufferPosition = handle->currBufferLength;
        handle->currHistoryReadPosition = handle->currHistoryWritePosition;
    }
}

char * strnchr(char * str, char c, uint32_t length){
    uint32_t currPos = 0;
    for(;currPos < length && str[currPos] != 0; currPos++){
        if(str[currPos] == c) return &str[currPos];
    }
    return NULL;
}

void strsft(char * src, int32_t startByte, int32_t offset){
    if(offset == 0) return;
    
    if(offset > 0){     //shift forward
        uint32_t currPos = strlen(src) + offset;
        src[currPos--] = 0;
        for(; currPos >= startByte; currPos--){
            if(currPos == 0){
                src[currPos] = ' ';
                break;
            }
            src[currPos] = src[currPos - offset];
        }
        return;
    }else{              //shift backward
        uint32_t currPos = startByte;
        for(; src[currPos - offset] != 0; currPos++){
            src[currPos] = src[currPos - offset];
        }
        src[currPos] = src[currPos - offset];
        return;
    }
}

TermCommandDescriptor * TERM_findCMD(TERMINAL_HANDLE * handle){
    uint8_t currPos = 0;
    uint16_t cmdLength = handle->currBufferLength;
    
    char * firstSpace = strchr(handle->inputBuffer, ' ');
    if(firstSpace != 0){
        cmdLength = (uint16_t) ((uint32_t) firstSpace - (uint32_t) handle->inputBuffer);
    }
    
    TermCommandDescriptor * currCmd = handle->cmdListHead->nextCmd;
    for(;currPos < handle->cmdListHead->commandLength; currPos++){
        if(currCmd->commandLength == cmdLength && strncmp(handle->inputBuffer, currCmd->command, cmdLength) == 0) return currCmd;
        currCmd = currCmd->nextCmd;
    }
    
    return 0;
}

uint8_t TERM_interpretCMD(char * data, uint16_t dataLength, TERMINAL_HANDLE * handle){
    
    TermCommandDescriptor * cmd = TERM_findCMD(handle);
    
    if(cmd != 0){
        uint16_t argCount = TERM_countArgs(data, dataLength);
        if(argCount == TERM_ARGS_ERROR_STRING_LITERAL){
            ttprintf("\r\nError: unclosed string literal in command\r\n");
            return TERM_CMD_EXIT_ERROR;
        }

        char ** args = 0;
        if(argCount != 0){
            args = pvPortMalloc(sizeof(char*) * argCount);
            TERM_seperateArgs(data, dataLength, args);
        }

        uint8_t retCode = TERM_CMD_EXIT_ERROR;
        if(cmd->function != 0){
            //configASSERT((cmd->function > 0x9d000000) && (cmd->function < 0x9d040000));
            retCode = (*cmd->function)(handle, argCount, args);
        }

        if(argCount != 0) vPortFree(args);
        return retCode;
    }
    
    return TERM_CMD_EXIT_NOT_FOUND;
}

uint16_t TERM_seperateArgs(char * data, uint16_t dataLength, char ** buff){
    uint8_t count = 0;
    uint8_t currPos = 0;
    unsigned quoteMark = 0;
    char * currStringStart = 0;
    char * lastSpace = 0;
    for(;currPos<dataLength; currPos++){
        switch(data[currPos]){
            case ' ':
                if(!quoteMark){
                    data[currPos] = 0;
                    lastSpace = &data[currPos + 1];
                }
                break;
                
            case '"':
                if(quoteMark){
                    quoteMark = 0;
                    if(currStringStart){
                        data[currPos] = 0;
                        buff[count++] = currStringStart;
                    }
                }else{
                    quoteMark = 1;
                    currStringStart = &data[currPos+1];
                }
                        
                break;
            default:
                if(!quoteMark){
                    if(lastSpace != 0){
                        buff[count++] = lastSpace;
                        lastSpace = 0;
                    }
                }
                break;
        }
    }
    if(quoteMark) return TERM_ARGS_ERROR_STRING_LITERAL;
    return count;
}

uint16_t TERM_countArgs(const char * data, uint16_t dataLength){
    uint8_t count = 0;
    uint8_t currPos = 0;
    unsigned quoteMark = 0;
    const char * currStringStart = NULL;
    const char * lastSpace = NULL;
    for(;currPos<dataLength; currPos++){
        switch(data[currPos]){
            case ' ':
                if(!quoteMark){
                    lastSpace = &data[currPos + 1];
                }
                break;
                
            case '"':
                if(quoteMark){
                    quoteMark = 0;
                    if(currStringStart){
                        count ++;
                    }
                }else{
                    quoteMark = 1;
                    currStringStart = &data[currPos+1];
                }
                        
                break;
            default:
                if(!quoteMark){
                    if(lastSpace){
                        count ++;
                        lastSpace = NULL;
                    }
                }
                break;
        }
    }
    if(quoteMark) return TERM_ARGS_ERROR_STRING_LITERAL;
    return count;
}

void TERM_freeCommandList(TermCommandDescriptor ** cl, uint16_t length){
    /*uint8_t currPos = 0;
    for(;currPos < length; currPos++){
        vPortFree(cl[currPos]);
    }*/
    vPortFree(cl);
}

TermCommandDescriptor * TERM_addCommand(TermCommandFunction function, const char * command, const char * description, uint8_t minPermissionLevel, TermCommandDescriptor * head){
    if(head->commandLength == 0xff) return 0;
    
    TermCommandDescriptor * newCMD = pvPortMalloc(sizeof(TermCommandDescriptor));
    
    newCMD->command = command;
    newCMD->commandDescription = description;
    newCMD->commandLength = strlen(command);
    newCMD->function = function;
    newCMD->ACHandler = 0;
    
    TERM_LIST_add(newCMD, head);
    return newCMD;
}

void TERM_LIST_add(TermCommandDescriptor * item, TermCommandDescriptor * head){
    uint32_t currPos = 0;
    TermCommandDescriptor ** lastComp = &head->nextCmd;
    TermCommandDescriptor * currComp = head->nextCmd;
    
    while(currPos < head->commandLength){
        if(TERM_isSorted(currComp, item)){
            *lastComp = item;
            item->nextCmd = currComp;
            head->commandLength ++;
            
            return;
        }
        if(currComp->nextCmd == 0){
            item->nextCmd = currComp->nextCmd;
            currComp->nextCmd = item;
            head->commandLength ++;
            return;
        }
        lastComp = &currComp->nextCmd;
        currComp = currComp->nextCmd;
    }
    
    item->nextCmd = 0;
    *lastComp = item;
    head->commandLength ++;
}


unsigned TERM_isSorted(TermCommandDescriptor * a, TermCommandDescriptor * b){
    uint8_t currPos = 0;
    //compare the lowercase ASCII values of each character in the command (They are alphabetically sorted)
    for(;currPos < a->commandLength && currPos < b->commandLength; currPos++){
        char letterA = toLowerCase(a->command[currPos]);
        char letterB = toLowerCase(b->command[currPos]);
        //if the letters are different we return 1 if a is smaller than b (they are correctly sorted) or zero if its the other way around
        if(letterA > letterB){
            return 1;
        }else if(letterB > letterA){
            return 0;
        }
    }
    
    //the two commands have identical letters for their entire length we check which one is longer (the shortest should come first)
    if(a->commandLength > b->commandLength){
        return 1;
    }else if(b->commandLength > a->commandLength){
        return 0;
    }else{
        //it might happen that a command is added twice (or two identical ones are added), in which case we just say they are sorted correctly and print an error in the console
        //TODO implement an alarm here
        //UART_print("WARNING: Found identical commands: \"%S\" and \"%S\"\r\n", a->command, b->command);        
        return 1;
    }
}

char toLowerCase(char c){
    if(c > 65 && c < 90){
        return c + 32;
    }

    return c;
}


void TERM_setCursorPos(TERMINAL_HANDLE * handle, uint16_t row, uint16_t column){
    ttprintf(ESC_STR "[%i;%iH", row, column);
}

void TERM_sendVT100Code(TERMINAL_HANDLE * handle, uint16_t cmd, uint8_t var){
    switch(cmd){
        case _VT100_RESET: //Keep
            ttprintf("%cc", 0x1b);
            break;
        case _VT100_CURSOR_BACK: //Keep
            ttprintf("\x1b[D");
            break;
        case _VT100_CURSOR_FORWARD:  //Keep
            ttprintf("\x1b[C");
            break;
        case _VT100_CURSOR_POS1:  //Keep
            ttprintf("\x1b[H");
            break;
        case _VT100_ERASE_LINE:  //Keep
            ttprintf("\x1b[2K");
            break;
        case _VT100_WRAP_OFF: //Keep
            ttprintf("\x1b[7l");
            break;
        case _VT100_ERASE_LINE_END: //Keep
            ttprintf("\x1b[K");
            break;
        case _VT100_CURSOR_BACK_BY: // //Keep
            ttprintf("\x1b[%dD", var);
            break;
        case _VT100_CURSOR_SET_COLUMN:  //Keep
            ttprintf(ESC_STR "[%i`", var);
            break;

    }
}

//returns a static pointer to the requested VT100 code, so it can be used in printf without needing any free() call afterwards
const char * TERM_getVT100Code(uint16_t cmd, uint8_t var){
    //INFO this is deprecated since all of the VT100 functions are handled by the terminal now
    switch(cmd){
        case _VT100_RESET:
            return "\x1b" "C";
            
        case _VT100_CURSOR_BACK:
            return "\x1b[D";
            
        case _VT100_CURSOR_FORWARD:
            return "\x1b[C";
            
        case _VT100_CURSOR_POS1:
            return "\x1b[H";
            
        case _VT100_CURSOR_END:
            return "\x1b[F";
        case _VT100_FOREGROUND_COLOR:
            switch(var){
                case _VT100_BLACK:
                    return "\x1b[30m";
                case _VT100_YELLOW:
                    return "\x1b[33m";
                case _VT100_BLUE:
                    return "\x1b[34m";
                case _VT100_WHITE:
                    return "\x1b[37m";
                default:
                    return "\x1b[30m";
            }
            
        case _VT100_BACKGROUND_COLOR:
            switch(var){
                case _VT100_WHITE:
                    return "\x1b[47m";
                default:
                    return "\x1b[40m";
            }
            
        case _VT100_RESET_ATTRIB:
            return "\x1b[0m";
            
        case _VT100_ERASE_SCREEN:
            return "\x1b[2J";
            
        case _VT100_ERASE_LINE:
            return "\x1b[2K";

        case _VT100_WRAP_OFF:
            return "\x1b[71";
            
        case _VT100_ERASE_LINE_END:
            return "\x1b[K";

        case _VT100_CURSOR_SET_COLUMN:
            return ESC_STR "[%i`";
            break;
            
    }
    return "";
}

void TERM_attachProgramm(TERMINAL_HANDLE * handle, TermProgram * prog){
    handle->currProgram = prog;
}

void TERM_removeProgramm(TERMINAL_HANDLE * handle){
    handle->currProgram = 0;
}


#ifdef TERM_ENABLE_STARTUP_TEXT
const char TERM_startupText1[] = "            333333    666   555555\r\nmm mm mmmm     3333  66     55\r\nmmm  mm  mm   3333  666666  555555";
const char TERM_startupText2[] = "mmm  mm  mm     333 66   66    5555\r\nmmm  mm  mm 333333   66666  555555\r\n";
const char TERM_startupText3[] = "\tBuild: " __DATE__ " - " __TIME__ "";
#endif

#endif
