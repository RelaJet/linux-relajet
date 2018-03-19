
#ifndef __CL_NCMEM_IOCTL_H__
#define __CL_NCMEM_IOCTL_H__

struct user_addr_info {
    unsigned int phys_addr;
    unsigned int user_addr;
};

#define CL_NCMEM_IOC_MAGIC		                    'n'
#define CL_NCMEM_IOC_ALLOCATE_NON_CACHE_MEMORTY    _IOW(CL_NCMEM_IOC_MAGIC, 0x00, unsigned int)
#define CL_NCMEM_IOC_FREE_NON_CACHE_MEMORTY        _IOW(CL_NCMEM_IOC_MAGIC, 0x01, unsigned long)
#define CL_NCMEM_IOC_SET_USER_ADDRESS_INFO         _IOW(CL_NCMEM_IOC_MAGIC, 0x10, struct user_addr_info)
#define CL_NCMEM_IOC_CHECK_MEMORY_OWNER            _IOW(CL_NCMEM_IOC_MAGIC, 0x20, unsigned int)

#define CL_NCMEM_IOC_GET_PHYS_ADDRESS_INFO         _IOW(CL_NCMEM_IOC_MAGIC, 0x11, unsigned int)

#endif
