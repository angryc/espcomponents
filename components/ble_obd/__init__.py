import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client
from esphome.const import CONF_ID

DEPENDENCIES = ["ble_client"]
CODEOWNERS = ["@angryc"]

ble_obd_ns = cg.esphome_ns.namespace("ble_obd")
BleObdComponent = ble_obd_ns.class_(
    "BleObdComponent", cg.Component, ble_client.BLEClientNode
)

CONF_BLE_CLIENT_ID = "ble_client_id"
CONF_SERVICE_UUID = "service_uuid"
CONF_RX_CHAR_UUID = "rx_char_uuid"
CONF_TX_CHAR_UUID = "tx_char_uuid"
CONF_INIT_COMMANDS = "init_commands"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BleObdComponent),
        cv.Required(CONF_BLE_CLIENT_ID): cv.use_id(ble_client.BLEClient),
        cv.Optional(CONF_SERVICE_UUID, default="FFE0"): cv.string,
        cv.Optional(CONF_RX_CHAR_UUID, default="FFE1"): cv.string,
        cv.Optional(CONF_TX_CHAR_UUID, default="FFE1"): cv.string,
        cv.Optional(CONF_INIT_COMMANDS, default=[]): cv.ensure_list(cv.string),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_BLE_CLIENT_ID])
    cg.add(parent.register_ble_node(var))

    cg.add(var.set_service_uuid(config[CONF_SERVICE_UUID]))
    cg.add(var.set_rx_char_uuid(config[CONF_RX_CHAR_UUID]))
    cg.add(var.set_tx_char_uuid(config[CONF_TX_CHAR_UUID]))

    for cmd in config[CONF_INIT_COMMANDS]:
        cg.add(var.add_init_command(cmd))
