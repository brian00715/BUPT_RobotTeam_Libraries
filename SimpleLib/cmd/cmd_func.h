#ifndef __CMD_FUNC_H
#define __CMD_FUNC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "simplelib_cfg.h"
#include "cmd.h"
#include "can_utils.h"
#include "can_func.h"
#include "flags.h"
#include "laser.h"
#include "main.h"
#include "point_parser.h"
#include "motor_driver.h"
#include "vesc_can.h"
#include <stdlib.h>
#include "dji_ctr.h"
#include <string.h>
#include "sw_chassis.h"

    void cmd_func_init(void);


#ifdef __cplusplus
}
#endif

#endif /* __CMD_FUNC_H */