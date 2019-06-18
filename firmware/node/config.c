#include "config.h"

#include <string.h>
#include "flash.h"

#define CONFIG_ADDR_CRC     (0x08008000 + 0x30000)
#define CONFIG_ADDR         (CONFIG_ADDR_CRC + 0x8)

struct config_item_t config_items[] = {
    {.name = "NODE id", .type = CONFIG_TYPE_INT, .val.i = CANARD_BROADCAST_NODE_ID, .def.i = CANARD_BROADCAST_NODE_ID, .min.i = 0, .max.i = CANARD_MAX_NODE_ID},
    {.name = "NODE timeout (ms)", .type = CONFIG_TYPE_INT, .val.i = 100, .def.i = 100, .min.i = 0, .max.i = 20000},
    {.name = "ESC index", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 254},
    {.name = "ESC failsafe", .type = CONFIG_TYPE_INT, .val.i = 1000, .def.i = 1000, .min.i = 750, .max.i = 2400},
    {.name = "SERVO type", .type = CONFIG_TYPE_INT, .val.i = 0, .def.i = 0, .min.i = 0, .max.i = 2},
    {.name = "SERVO index", .type = CONFIG_TYPE_INT, .val.i = 1, .def.i = 1, .min.i = 0, .max.i = 254},
    {.name = "SERVO P-gain", .type = CONFIG_TYPE_INT, .val.i = 75, .def.i = 75, .min.i = 0, .max.i = 100},
    {.name = "SERVO D-gain", .type = CONFIG_TYPE_INT, .val.i = 40, .def.i = 40, .min.i = 0, .max.i = 100},
    {.name = "SERVO max-power", .type = CONFIG_TYPE_INT, .val.i = 80, .def.i = 80, .min.i = 0, .max.i = 100},
    {.name = "SERVO stall-power", .type = CONFIG_TYPE_INT, .val.i = 45, .def.i = 45, .min.i = 0, .max.i = 100},
};
static uint16_t config_crc = 0;
const uint8_t config_items_cnt = sizeof(config_items) / sizeof(struct config_item_t);

static uint16_t calc_crc(struct config_item_t *config_items, uint8_t len) {
    uint8_t *data = (uint8_t *)config_items;
    uint32_t size = sizeof(struct config_item_t) * len;
    uint16_t crc = 0xFFFF;

    for(uint32_t i = 0; i < size; i++) {
        crc = ((data[i] << 8) ^ crc);

        for(uint8_t j = 0; j < 8; j++) {
            if(crc & 0x8000)
                crc = (crc << 1) ^ 0x8005;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

void config_init(void) {
    config_crc = calc_crc(config_items, config_items_cnt);
    uint16_t flash_crc = *(uint16_t *)(CONFIG_ADDR_CRC);

    if(flash_crc != config_crc) {
        config_reset();
        config_save();
    }
    else {
        config_read();
    }
}

void config_save(void) {
    union config_val_t values[config_items_cnt];
    uint32_t size = sizeof(union config_val_t) * config_items_cnt;

    // Copy the values for saving
    for(uint8_t i = 0; i < config_items_cnt; i++)
        values[i] = config_items[i].val;

    flash_erase_pages(CONFIG_ADDR_CRC, 2 + size);
    flash_write_block((void *)CONFIG_ADDR_CRC, (uint8_t *)&config_crc, 2);
    flash_write_block((void *)CONFIG_ADDR, (uint8_t *)values, size);
}

void config_read(void) {
    union config_val_t *values = (union config_val_t *)CONFIG_ADDR;

    // Copy the values for loading
    for(uint8_t i = 0; i < config_items_cnt; i++)
        config_items[i].val = values[i];
}

void config_reset(void) {
    // Reset to default values
    for(uint8_t i = 0; i < config_items_cnt; i++)
        config_items[i].val = config_items[i].def;
}

/* Get a config item by name */
struct config_item_t *config_get_by_name(char* name, uint8_t len) {
    if(len == 0)
        len = strlen(name);
    
    // Search the config
    for(uint8_t i = 0; i < config_items_cnt; i++) {
        if(strncmp(config_items[i].name, name, len) == 0)
            return &config_items[i];
    }

    return NULL;
}

static void config_set_resp(struct config_item_t *item, uavcan_protocol_param_GetSetResponse *resp) {
    resp->name.data = (uint8_t *)item->name;
    resp->name.len = strlen(item->name);

    switch(item->type) {
        case CONFIG_TYPE_EMPTY:
            resp->value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
            resp->default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
            break;
        case CONFIG_TYPE_INT:
            resp->value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            resp->value.integer_value = item->val.i;
            resp->default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            resp->default_value.integer_value = item->def.i;
            resp->min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
            resp->min_value.integer_value = item->min.i;
            resp->max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
            resp->max_value.integer_value = item->max.i;
            break;
        case CONFIG_TYPE_FLOAT:
            resp->value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
            resp->value.real_value = item->val.f;
            resp->default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
            resp->default_value.real_value = item->def.f;
            resp->min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
            resp->min_value.real_value = item->min.f;
            resp->max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
            resp->max_value.real_value = item->max.f;
            break;
        case CONFIG_TYPE_BOOL:
            resp->value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
            resp->value.boolean_value = item->val.b;
            resp->default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
            resp->default_value.boolean_value = item->def.b;
            break;
        case CONFIG_TYPE_STRING:
            resp->value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE;
            resp->value.string_value.data = (uint8_t *)item->val.s;
            resp->value.string_value.len = (uint8_t)strlen(item->val.s);
            resp->default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE;
            resp->default_value.string_value.data = (uint8_t *)item->def.s;
            resp->default_value.string_value.len = (uint8_t)strlen(item->def.s);
            break;
    }
}

/*
  handle a PARAM_GETSET request
 */
void handle_param_getset(struct uavcan_iface_t *iface, CanardRxTransfer* transfer)
{
    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uavcan_protocol_param_GetSetResponse resp = {0};
    uavcan_protocol_param_GetSetRequest req = {0};

    // Decode the incoming request
    uint8_t dyn_arr_buff[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_NAME_MAX_LENGTH+1];
    uint8_t *dyn1 = dyn_arr_buff;
	if(uavcan_protocol_param_GetSetRequest_decode(transfer, transfer->payload_len, &req, &dyn1) < 0)
        return;
    
    // Get the correct config item
    struct config_item_t *item = NULL;
    if(req.name.data && req.name.len) {
        item = config_get_by_name((char *)req.name.data, req.name.len);
    } else if (req.index < config_items_cnt) {
        item = &config_items[req.index];
    }

    // If we did find the config item
    if(item != NULL) {
        // We want to nupdate the value
        if (req.value.union_tag == item->type) {
            switch(item->type) {
                case CONFIG_TYPE_INT:
                    item->val.i = req.value.integer_value;
                    resp.value.union_tag = item->type;
                    resp.value.integer_value = item->val.i;
                    break;
                default:
                    break;
            }
        }

        // Set the response
        config_set_resp(item, &resp);
    }
    // The config item was not found
    else {
        uint8_t name[1];
        resp.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
        resp.name.data = name;
        resp.name.len = 0;
    }

    uint16_t total_size = uavcan_protocol_param_GetSetResponse_encode(&resp, buffer);
    uavcanRequestOrRespond(iface,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           total_size);
}

void handle_param_execute_opcode(struct uavcan_iface_t *iface, CanardRxTransfer* transfer) {
    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    uavcan_protocol_param_ExecuteOpcodeResponse resp = {0};
    uavcan_protocol_param_ExecuteOpcodeRequest req = {0};

    if(uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, transfer->payload_len, &req, NULL) < 0)
        return;

    resp.ok = false;
    if(req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        config_reset();
        config_save();
        resp.ok = true;
    } else if(req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
        config_save();
        resp.ok = true;
    }

    uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&resp, buffer);
    uavcanRequestOrRespond(iface,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           total_size);
}