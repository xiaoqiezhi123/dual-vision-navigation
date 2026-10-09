// commands.h
#ifndef COMMANDS_H
#define COMMANDS_H
#ifdef __cplusplus
extern "C" {
#endif

#include "global_options.h"

int execute_command(const char *command_name, GlobalOptions *opts, int argc, char *argv[]);

#ifdef __cplusplus
}
#endif
#endif
