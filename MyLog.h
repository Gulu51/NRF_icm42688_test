#ifndef __MY_LOG_H__
#define __MY_LOG_H__

#include "SEGGER_RTT.h"

#define MY_LOG_INFO(fmt, ...)  SEGGER_RTT_printf(0 ,RTT_CTRL_TEXT_BRIGHT_GREEN"<Info>""[%s|%d] "fmt"\r\n",__FILE__,__LINE__,##__VA_ARGS__)
#define MY_LOG_DEBUG(fmt, ...)  SEGGER_RTT_printf(0 ,RTT_CTRL_TEXT_BRIGHT_WHITE"<Debug>""[%s|%d] "fmt"\r\n",__FILE__,__LINE__,##__VA_ARGS__)
#define MY_LOG_WARNING(fmt, ...)  SEGGER_RTT_printf(0 ,RTT_CTRL_TEXT_BRIGHT_YELLOW"<Warning>""[%s|%d] "fmt"\r\n",__FILE__,__LINE__,##__VA_ARGS__)
#define MY_LOG_ERROR(fmt, ...)  SEGGER_RTT_printf(0 ,RTT_CTRL_TEXT_BRIGHT_RED"<Error>""[%s|%d] "fmt"\r\n",__FILE__,__LINE__,##__VA_ARGS__)


// //字符串宏
// #define _STRCAT2(a,b) a##b
// #define STRCAT2(a,b) _STRCAT2(a,b)
// //拼接宏
// #define _STR(x) #x
// #define STR(x) _STR(x)

/* RTT接收终端输入的范例rtt参数0，是至0channel，不是指0terminal
uint8_t buff[128] = {0};
if(SEGGER_RTT_HasData(0) != false) 
{
    SEGGER_RTT_Read(0,buff,5);
    if(strncmp((const char*)buff,"hello",5) == 0)
    {
        NRF_LOG_INFO("Hi This SEGGER Jlink");
    }
    memset(buff, 0 , 5);
}
*/

/* RTT改变channel0输出到terminal1
SEGGER_RTT_SetTerminal(1);
*/

#endif
