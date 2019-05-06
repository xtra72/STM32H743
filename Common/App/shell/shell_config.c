#include "target.h"
#include "config.h"
#include "shell.h"
#include "shell_rf.h"
#include "shell_scan.h"

extern  CONFIG  config_;

RET_VALUE SHELL_config(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    RET_VALUE   ret = RET_INVALID_ARGUMENT;

    switch(argc)
    {
    case    1:
        {
            SHELL_printf("\n%s\n", "[ System ]");
            SHELL_printf("%16s : %s\n", "S/N", config_.serialNumber);

            SHELL_printf("\n%s\n", "[ Debug ]");
            SHELL_printf("%16s : %s\n", "Trace", TRACE_getEnable()?"ON":"OFF");

            SHELL_printf("\n%s\n", "[ RF ]");
            SHELL_RF_info();
#if SUPPORT_DRAM
            SHELL_printf("\n%s\n", "[ SCAN ]");
            SHELL_SCAN_info();
#endif
            ret = RET_OK;
        }
        break;

    case    2:
        {
            if (strcasecmp(argv[1], "save") == 0)
            {
                CONFIG* config = pvPortMalloc(sizeof(CONFIG));
                if (config == NULL)
                {
                    ret = RET_NOT_ENOUGH_MEMORY;
                    break;
                }

                ret = RF_getConfig(&config->rf);
                if (ret != RET_OK)
                {
                    vPortFree(config);
                    SHELL_printf("Can't get RF settings!\n");
                    break;
                }

                ret = SHELL_getConfig(&config->shell);
                if (ret != RET_OK)
                {
                    vPortFree(config);
                    SHELL_printf("Can't get shell settings!\n");
                    break;
                }

                ret = TRACE_getConfig(&config->trace);
                if (ret != RET_OK)
                {
                    vPortFree(config);
                    SHELL_printf("Can't get TRACE settings!\n");
                    break;
                }

                ret = CONFIG_save(config);
                if (ret != RET_OK)
                {
                    vPortFree(config);
                    SHELL_printf("Failed to save settings!\n");
                    break;
                }

                memcpy(&config_, config, sizeof(CONFIG));
                vPortFree(config);
            }
            else if (strcasecmp(argv[1], "load") == 0)
            {
                ret = CONFIG_load(&config_);
            }
            else if (strcasecmp(argv[1], "clean") == 0)
            {
                ret = CONFIG_clear();
            }
        }
        break;


    default:
        ret = RET_INVALID_ARGUMENT;
    }


    return  ret;
}
