#ifndef SHELL_COMMAND_H_
#define SHELL_COMMAND_H_

RET_VALUE SHELL_COMMAND_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_COMMAND_getVersion(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_COMMAND_serialNumber(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_COMMAND_date(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_COMMAND_config(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

RET_VALUE SHELL_COMMAND_status(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_COMMAND_wakeUp(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_COMMAND_reset(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_COMMAND_receive(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_COMMAND_at(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_COMMAND_autoSleep(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_COMMAND_socket(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

RET_VALUE SHELL_COMMAND_bat(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_COMMAND_sleep(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_COMMAND_alarm(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_COMMAND_extPower(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_COMMAND_reset(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

RET_VALUE SHELL_COMMAND_trace(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

#endif
