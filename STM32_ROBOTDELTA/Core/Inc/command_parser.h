#ifndef __COMMAND_PARSER_H
#define __COMMAND_PARSER_H

#include "main.h"

/**
 * @brief Phân tích một chuỗi lệnh nhận được từ PC.
 *
 * @param command_string Chuỗi lệnh đầy đủ, không bao gồm ký tự xuống dòng.
 */
void parse_command(char* command_string);
void send_status_report(void);


#endif // __COMMAND_PARSER_H