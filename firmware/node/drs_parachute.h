#ifndef DRS_PARACHUTE_H
#define DRS_PARACHUTE_H

#include <hal.h>

enum parachute_status_t {
    DRS_STATUS_INIT,
    DRS_STATUS_DISABLE,
    DRS_STATUS_ENABLE,
    DRS_STATUS_RELEASE
};

struct drs_parachute_t {
    uint8_t index;                      ///< Aatuator index
    UARTDriver *port;                   ///< Serial port
    enum parachute_status_t status;     ///< Last parachute status
    UARTConfig uart_cfg;                ///< UART configuration
};
extern struct drs_parachute_t drs_parachute;

void drs_parachute_init(void);
void drs_parachute_set(enum parachute_status_t status);

#endif /* DRS_PARACHUTE_H */