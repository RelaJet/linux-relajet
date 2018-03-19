#ifndef __MMP_CPU_COMM_H__
#define __MMP_CPU_COMM_H__

#include "mmp_reg_int.h"
#include "mmp_register.h"

typedef enum __CPU_ID
{
    _CPU_ID_A = 0,
    _CPU_ID_B = 1
} _CPU_ID;

static __inline void MMP_CPUCOMM_IRQ_SET( _CPU_ID ulId )
{
    AITC_BASE_HINT->HINT_SET_CPU_INT = (ulId==_CPU_ID_A)? HINT_CPU2CPUA_INT : HINT_CPU2CPUB_INT;
}

static __inline void MMP_CPUCOMM_IRQ_CLEAR( _CPU_ID ulId )
{
    AITC_BASE_HINT->HINT_CLR_CPU_INT = (ulId==_CPU_ID_A)? HINT_CPU2CPUA_INT : HINT_CPU2CPUB_INT;
}

static __inline MMP_BOOL MMP_CPUCOMM_IRQ_CHECK( AITPS_HINT psHintBase, _CPU_ID ulId )
{
    return psHintBase->HINT_CTL &( (ulId==_CPU_ID_A)? HINT_CPU2CPUA_INT : HINT_CPU2CPUB_INT );
}

static __inline AIT_REG_B * MMP_CPUCOMM_SHARE_REG_GET(void)
{
    return AITC_BASE_CPU_SAHRE->CPU_SAHRE_REG;
}
#endif //__MMP_CPU_COMM_H__
