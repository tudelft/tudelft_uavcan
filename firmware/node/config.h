#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
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
  char s[8];
};

struct config_item_t {
  char name[32];
  enum config_type_t type;

  union config_val_t val;
  union config_val_t def;
  union config_num_t min;
  union config_num_t max;
};

extern struct config_item_t config_items[];
extern const uint8_t config_items_cnt;
extern uint16_t config_crc;
extern uint16_t config_crc_r1;
extern uint16_t config_crc_r2;

extern void config_init(void);
extern void config_save(void);
extern void config_read(void);
extern void config_reset(void);
extern struct config_item_t *config_get_by_name(char* name, uint8_t len);
int16_t config_get_i16(const char *name, int16_t default_value);
uint8_t config_get_u8(const char *name, uint8_t default_value);
uint16_t config_get_u16(const char *name, uint16_t default_value);
uint32_t config_get_u32(const char *name, uint32_t default_value);
float config_get_f32(const char *name, float default_value);
bool config_get_bool(const char *name, bool default_value);
void handle_param_getset(struct uavcan_iface_t *iface, CanardRxTransfer* transfer);
void handle_param_execute_opcode(struct uavcan_iface_t *iface, CanardRxTransfer* transfer);

#endif /* CONFIG_H */