/*
 * cli_setup.c
 *
 *  Created on: Apr 1, 2025
 *      Author: HTSANG
 */

#include "cli_setup.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "cli_setup.h"
#include "cli_command.h"
#include "uart_driver.h"

// Expand cli implementation here (must be in one file only)
#define EMBEDDED_CLI_IMPL

#include "embedded_cli.h"

// CLI buffer
/*************************************************
 *           CLI Static Buffer Define            *
 *************************************************/

//#define EXPFORWARD_CLI_BUFFER_SIZE 2048
//static  CLI_UINT EXPForward_cliStaticBuffer[BYTES_TO_CLI_UINTS(EXPFORWARD_CLI_BUFFER_SIZE)];
#define EXPCONSOLE_CLI_BUFFER_SIZE 2048
static  CLI_UINT EXPConsole_cliStaticBuffer[BYTES_TO_CLI_UINTS(EXPCONSOLE_CLI_BUFFER_SIZE)];


/*************************************************
 *             ----------------------            *
 *************************************************/

//static EmbeddedCli *cli_EXPForward;
static EmbeddedCli *cli_EXPConsole;

// Bool to disable the interrupts, if CLI is not yet ready.
static _Bool cliIsReady = false;


/*************************************************
 *          Tx Transmit CLI Byte Buffer          *
 *************************************************/

//static void writeCharTocli_EXPForward(EmbeddedCli *embeddedCli, char c) {
//    uint8_t c_to_send = c;
//    UART_WriteRing(EXP_UART_FORWARD_HANDLE, c_to_send);
//}

static void writeCharTocli_EXPConsole(EmbeddedCli *embeddedCli, char c) {
    uint8_t c_to_send = c;
    UART_Driver_Write(EXP_UART_CONSOLE_HANDLE, c_to_send);
}

Std_ReturnType SystemCLI_Init() {
	/* Initialize the CLI configuration settings */

//	// Initialize UART EXP FORWARD CLI
//    EmbeddedCliConfig *EXPForward_config = embeddedCliDefaultConfig();
//    EXPForward_config->cliBuffer = EXPForward_cliStaticBuffer;
//    EXPForward_config->cliBufferSize = EXPFORWARD_CLI_BUFFER_SIZE;
//    EXPForward_config->rxBufferSize = CLI_RX_BUFFER_SIZE;
//    EXPForward_config->cmdBufferSize = CLI_CMD_BUFFER_SIZE;
//    EXPForward_config->historyBufferSize = CLI_HISTORY_SIZE;
//    EXPForward_config->maxBindingCount = CLI_MAX_BINDING_COUNT;
//    EXPForward_config->enableAutoComplete = CLI_AUTO_COMPLETE;
//    EXPForward_config->invitation = CLI_INITATION_EXP_FORWARD;
//    EXPForward_config->staticBindings = getCliStaticBindings();
//    EXPForward_config->staticBindingCount = getCliStaticBindingCount();
//
//    cli_EXPForward = embeddedCliNew(EXPForward_config);
//    if (cli_EXPForward == NULL) {
//        return E_ERROR;
//    }
//    cli_EXPForward->writeChar = writeCharTocli_EXPForward;

    // Initialize UART EXP CONSOLE CLI
    EmbeddedCliConfig *EXPConsole_config = embeddedCliDefaultConfig();
    EXPConsole_config->cliBuffer = EXPConsole_cliStaticBuffer;
    EXPConsole_config->cliBufferSize = EXPCONSOLE_CLI_BUFFER_SIZE;
    EXPConsole_config->rxBufferSize = CLI_RX_BUFFER_SIZE;
    EXPConsole_config->cmdBufferSize = CLI_CMD_BUFFER_SIZE;
    EXPConsole_config->historyBufferSize = CLI_HISTORY_SIZE;
    EXPConsole_config->maxBindingCount = CLI_MAX_BINDING_COUNT;
    EXPConsole_config->enableAutoComplete = CLI_AUTO_COMPLETE;
    EXPConsole_config->invitation = CLI_INITATION_EXP_CONSOLE;
    EXPConsole_config->staticBindings = getCliStaticBindings();
    EXPConsole_config->staticBindingCount = getCliStaticBindingCount();

    cli_EXPConsole = embeddedCliNew(EXPConsole_config);
    if (cli_EXPConsole == NULL) {
        return E_ERROR;
    }
    cli_EXPConsole->writeChar = writeCharTocli_EXPConsole;

//	// Init the CLI with blank screen
//	onClearCLI(cli, NULL, NULL);

    // CLI has now been initialized, set bool to true to enable interrupts.
    cliIsReady = true;

    return E_OK;
}


/*************************************************
 *             Get CLI Pointers                  *
 *************************************************/
//EmbeddedCli *get_EXPForward_CliPointer() {
//    return cli_EXPForward;
//}

EmbeddedCli *get_EXPConsole_CliPointer() {
    return cli_EXPConsole;
}
