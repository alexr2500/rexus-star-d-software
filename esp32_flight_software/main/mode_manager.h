#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

typedef enum{
    MODE_SU,
    MODE_T,
    MODE_NG,
    MODE_NF,
    MODE_NE
} software_mode_t;

software_mode_t software_mode_get(void);

#endif