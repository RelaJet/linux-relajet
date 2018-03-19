
#include <linux/module.h>
#include <linux/spinlock.h>

#include <mach/includes_fw.h>

#include <mach/mmpf_system.h>
#include <mach/mmpf_pll.h>
#include <mach/mmpf_i2cm.h>

#include <mach/ait_if.h>



int ait_param_ctrl(enum AIT_PARAM param, void *arg)
{

	static struct ait_cam_interface cam_if = {
	    .lock           = __SPIN_LOCK_UNLOCKED(.lock),
	    .registered     = 0,
	    .ir_state       = 0,
	    .ir_notify      = 0,
	    .priv           = 0
	};


    unsigned long flags;

    switch (param) {
    case AIT_PARAM_CAM_IR_S_CTRL:
        spin_lock_irqsave(&cam_if.lock, flags);
        cam_if.ir_state = ((struct ait_camif_ir_ctrl*)arg)->ir_on;
        if (cam_if.registered && cam_if.ir_notify) {
            cam_if.ir_notify(cam_if.priv);
        }
        spin_unlock_irqrestore(&cam_if.lock, flags);
        break;
    case AIT_PARAM_CAM_IR_G_CTRL:
        ((struct ait_camif_ir_ctrl*)arg)->ir_on = cam_if.ir_state;
        break;
    case AIT_PARAM_CAM_IF_REG:
        spin_lock_irqsave(&cam_if.lock, flags);
        cam_if.registered   = 1;
        cam_if.ir_notify    = ((struct ait_camif_reg_ctrl*)arg)->ir_notify;
        cam_if.priv         = ((struct ait_camif_reg_ctrl*)arg)->priv;
        spin_unlock_irqrestore(&cam_if.lock, flags);
        break;
    case AIT_PARAM_CAM_IF_FREE:
        spin_lock_irqsave(&cam_if.lock, flags);
        cam_if.registered   = 0;
        cam_if.ir_notify    = 0;
        cam_if.priv         = 0;
        spin_unlock_irqrestore(&cam_if.lock, flags);
        break;
    default:
        return -EINVAL;
    }

    return 0;
}
EXPORT_SYMBOL(ait_param_ctrl);


/*
 * Exported Symbols From MMPF drivers
 */

EXPORT_SYMBOL(MMPF_SYS_ResetHModule);
EXPORT_SYMBOL(MMPF_SYS_EnableClock);
EXPORT_SYMBOL(MMPF_SYS_TuneMCIPriority);
EXPORT_SYMBOL(MMPF_SYS_SetSensorInputCapability);

EXPORT_SYMBOL(MMPF_PLL_GetCPUFreq);
EXPORT_SYMBOL(MMPF_PLL_GetGroupFreq);

EXPORT_SYMBOL(MMPF_I2cm_ReadReg);
EXPORT_SYMBOL(MMPF_I2cm_WriteReg);



