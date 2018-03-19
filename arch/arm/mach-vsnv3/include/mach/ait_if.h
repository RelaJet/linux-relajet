
#ifndef _AIT_IF_H_
#define _AIT_IF_H_



enum AIT_PARAM {
    AIT_PARAM_CAM_IR_S_CTRL,
    AIT_PARAM_CAM_IR_G_CTRL,
    AIT_PARAM_CAM_IF_REG,
    AIT_PARAM_CAM_IF_FREE
};

struct ait_camif_ir_ctrl {
    int     ir_on;                  ///< 1: IR mode on; 0: IR off
};

struct ait_camif_reg_ctrl {
    void    (*ir_notify)(void*);
    void    *priv;
};

struct ait_cam_interface {
    void            *priv;
    spinlock_t      lock;
    bool            registered;

    int             ir_state;
    void            (*ir_notify)(void*);
};


int ait_param_ctrl (enum AIT_PARAM param, void *arg);


#endif //#ifndef _AIT_IF_H_
