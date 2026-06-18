import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID
from .. import ble_obd_ns, BleObdComponent, BleObdDevice

DEPENDENCIES = ["ble_obd"]

BleObdSensor = ble_obd_ns.class_(
    "BleObdSensor", sensor.Sensor, BleObdDevice
)

CONF_BLE_OBD_ID = "ble_obd_id"
CONF_PID = "pid"
CONF_MODE = "mode"
CONF_PRE_COMMANDS = "pre_commands"
CONF_FORMULA = "formula"

def _validate_pid(config):
    if CONF_PID not in config:
        raise cv.Invalid("'pid' must be specified")
    return config

CONFIG_SCHEMA = cv.All(
    _validate_pid,
    sensor.sensor_schema(BleObdSensor).extend(
        {
            cv.GenerateID(CONF_BLE_OBD_ID): cv.use_id(BleObdComponent),
            cv.Required(CONF_PID): cv.string,
            cv.Optional(CONF_MODE, default="01"): cv.string,
            cv.Optional(CONF_PRE_COMMANDS, default=[]): cv.ensure_list(cv.string),
            cv.Optional(CONF_FORMULA): cv.returning_lambda,
        }
    ).extend(cv.polling_component_schema("60s")),
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)

    paren = await cg.get_variable(config[CONF_BLE_OBD_ID])
    cg.add(paren.add_device(var))
    cg.add(var.set_parent(paren))

    cg.add(var.set_pid(config[CONF_PID]))
    cg.add(var.set_mode(config[CONF_MODE]))

    for cmd in config[CONF_PRE_COMMANDS]:
        cg.add(var.add_pre_command(cmd))

    if CONF_FORMULA in config:
        formula_ = await cg.process_lambda(
            config[CONF_FORMULA],
            [
                (cg.uint8, "a"), (cg.uint8, "b"), (cg.uint8, "c"), (cg.uint8, "d"),
                (cg.uint8, "e"), (cg.uint8, "f"), (cg.uint8, "g"), (cg.uint8, "h"),
                (cg.uint8, "i"), (cg.uint8, "j"), (cg.uint8, "k"), (cg.uint8, "l"),
                (cg.uint8, "m"), (cg.uint8, "n"), (cg.uint8, "o"), (cg.uint8, "p"),
                (cg.uint8, "q"), (cg.uint8, "r"), (cg.uint8, "s"), (cg.uint8, "t"),
                (cg.uint8, "u"), (cg.uint8, "v"), (cg.uint8, "w"), (cg.uint8, "x"),
            ],
            return_type=cg.float_,
        )
        cg.add(var.set_formula(formula_))
