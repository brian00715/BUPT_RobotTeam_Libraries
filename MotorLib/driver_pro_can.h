#ifndef DRIVER_PRO_CAN_H_
#define DRIVER_PRO_CAN_H_

#ifdef USE_MTR_DRIVER_DRIVER_PRO

#include "can_utils.h"
#include "motor_driver.h"

void DrivePro_SetDuty(MotorDriver_s drivepro);
void DrivePro_SetSpeed(MotorDriver_s drivepro);

#endif // USE_MTR_DRIVER_DRIVER_PRO

#endif