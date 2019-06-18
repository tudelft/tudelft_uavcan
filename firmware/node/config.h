#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include "uavcan.h"

enum config_type_t {
  CONFIG_TYPE_EMPTY,
  CONFIG_TYPE_INT,
  CONFIG_TYPE_FLOAT,
  CONFIG_TYPE_BOOL,
  CONFIG_TYPE_STRING
};

union config_num_t {
  int64_t i;
  float f;
};

union config_val_t {
  int64_t i;
  float f;
  uint8_t b;
  char s[128];
};

struct config_item_t {
  char name[92];
  enum config_type_t type;

  union config_val_t val;
  union config_val_t def;
  union config_num_t min;
  union config_num_t max;
};

extern struct config_item_t config_items[];
extern const uint8_t config_items_cnt;

extern void config_init(void);
extern void config_save(void);
extern void config_read(void);
extern void config_reset(void);
extern struct config_item_t *config_get_by_name(char* name, uint8_t len);
void handle_param_getset(struct uavcan_iface_t *iface, CanardRxTransfer* transfer);
void handle_param_execute_opcode(struct uavcan_iface_t *iface, CanardRxTransfer* transfer);

#endif /* CONFIG_H */