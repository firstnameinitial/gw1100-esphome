# SPDX-License-Identifier: GPL-3.0-only
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = []

ecowitt_rf_ns = cg.esphome_ns.namespace("ecowitt_rf")

CONF_DEVICE_TYPE = "device_type"
CONF_ECOWITT_IP = "ecowitt_ip"
CONF_ECOWITT_PATH = "ecowitt_path"
CONF_ECOWITT_PORT = "ecowitt_port"
CONF_PASSKEY = "passkey"
CONF_STATIONTYPE = "stationtype"
CONF_FREQ = "freq"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(cg.Component),
        cv.Optional(CONF_DEVICE_TYPE, default=1): cv.int_range(0, 3),  # 0=433,1=868,2=915,3=920
        cv.Optional(CONF_ECOWITT_IP, default=""): cv.string,
        cv.Optional(CONF_ECOWITT_PATH, default="/data/report/"): cv.string,
        cv.Optional(CONF_ECOWITT_PORT, default=80): cv.port,
        cv.Optional(CONF_PASSKEY, default=""): cv.string,
        cv.Optional(CONF_STATIONTYPE, default="GW1100-ESPHome"): cv.string,
        cv.Optional(CONF_FREQ, default="868M"): cv.one_of("433M", "868M", "915M", "920M"),
    }
)
