# SPDX-License-Identifier: GPL-3.0-only
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, switch, text_sensor, time
from esphome.const import (
    CONF_ID,
    CONF_TEMPERATURE,
    CONF_HUMIDITY,
    CONF_PRESSURE,
    CONF_CO2,
    CONF_PM_2_5,
    CONF_PM_10_0,
    CONF_LIGHT,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_PRESSURE,
    DEVICE_CLASS_PM25,
    DEVICE_CLASS_PM10,
    DEVICE_CLASS_IRRADIANCE,
    DEVICE_CLASS_MOISTURE,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_PRECIPITATION,
    DEVICE_CLASS_PRECIPITATION_INTENSITY,
    DEVICE_CLASS_SPEED,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_HECTOPASCAL,
    UNIT_PARTS_PER_MILLION,
    UNIT_MICROGRAMS_PER_CUBIC_METER,
    UNIT_DEGREES,
    UNIT_MILLIMETER,
    UNIT_KILOMETER,
)

CONF_DEVICE_TYPE = "device_type"
CONF_ECOWITT_IP = "ecowitt_ip"
CONF_ECOWITT_PATH = "ecowitt_path"
CONF_ECOWITT_PORT = "ecowitt_port"
CONF_PASSKEY = "passkey"
CONF_STATIONTYPE = "stationtype"
CONF_FREQ = "freq"
# The primary outdoor array's own T/H. Kept separate from CONF_TEMPERATURE/CONF_HUMIDITY,
# which belong to the gateway's built-in I2C sensor — sharing them made the indoor entity
# alternate between the indoor and outdoor readings.
CONF_OUTDOOR_TEMPERATURE = "outdoor_temperature"
CONF_OUTDOOR_HUMIDITY = "outdoor_humidity"
CONF_WIND_SPEED = "wind_speed"
CONF_WIND_DIR = "wind_direction"
CONF_GUST = "wind_gust"
CONF_RAIN = "rain"
CONF_RAIN_RATE = "rain_rate"
CONF_UV = "uv_index"
CONF_LIGHT = "solar_radiation"
CONF_SOIL = "soil_moisture"
CONF_SOIL_TEMP = "soil_temperature"
CONF_SOIL_EC = "soil_ec"
CONF_PM25 = "pm25"
CONF_PM10 = "pm10"
CONF_CO2 = "co2"
CONF_LEAF = "leaf_wetness"
CONF_LIGHTNING_STRIKES = "lightning_strikes"
CONF_LIGHTNING_DISTANCE = "lightning_distance"
CONF_LEAK = "leak"
CONF_TIME_ID = "time_id"
CONF_ROSTER = "sensor_roster"
CONF_DEVICE_INFO = "device_info"
CONF_I2C_ID = "i2c_id"
CONF_UPLOAD_SWITCH = "upload_switch"
CONF_FAMILY_SWITCHES = "family_switches"

ecowitt_rf_ns = cg.esphome_ns.namespace("ecowitt_rf")
EcowittRFSensor = ecowitt_rf_ns.class_("EcowittRFSensor", cg.PollingComponent)

# Pull in the entity components this platform creates (text_sensor for the roster +
# device-info; time for the runtime-timezone apply) and web_server_base (the shared
# ESPAsyncWebServer the custom dashboard registers its / /state /config routes on).
AUTO_LOAD = ["text_sensor", "time", "web_server_base", "json"]

def validate_remote_channels(value):
    for key in ("sensor_id", "channel"):
        if len({item[key] for item in value}) != len(value):
            raise cv.Invalid(f"remote_indoor_channels requires unique {key} values")
    return value


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(EcowittRFSensor),
        cv.Optional("remote_indoor_channels", default=[]): cv.All(
            cv.ensure_list(cv.Schema({
                cv.Required("sensor_id"): cv.int_range(min=0, max=0xFFFFFF),
                cv.Required("channel"): cv.int_range(min=1, max=8),
            })), validate_remote_channels),
        cv.Optional(CONF_DEVICE_TYPE, default=1): cv.int_range(0, 3),
        cv.Optional(CONF_ECOWITT_IP, default=""): cv.string,
        cv.Optional(CONF_ECOWITT_PATH, default="/data/report/"): cv.string,
        cv.Optional(CONF_ECOWITT_PORT, default=80): cv.port,
        cv.Optional(CONF_PASSKEY, default=""): cv.string,
        cv.Optional(CONF_STATIONTYPE, default="GW1100-ESPHome"): cv.string,
        cv.Optional(CONF_FREQ, default="868M"): cv.one_of("433M", "868M", "915M", "920M"),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS, accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT, accuracy_decimals=0,
            device_class=DEVICE_CLASS_HUMIDITY, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_PRESSURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_HECTOPASCAL, accuracy_decimals=1,
            device_class=DEVICE_CLASS_PRESSURE, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_OUTDOOR_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS, accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_OUTDOOR_HUMIDITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT, accuracy_decimals=0,
            device_class=DEVICE_CLASS_HUMIDITY, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_WIND_SPEED): sensor.sensor_schema(
            unit_of_measurement="m/s", accuracy_decimals=1,
            device_class=DEVICE_CLASS_SPEED, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_WIND_DIR): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES, accuracy_decimals=0, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_GUST): sensor.sensor_schema(
            unit_of_measurement="m/s", accuracy_decimals=1,
            device_class=DEVICE_CLASS_SPEED, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_RAIN): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER, accuracy_decimals=2,
            device_class=DEVICE_CLASS_PRECIPITATION, state_class=STATE_CLASS_TOTAL_INCREASING),
        cv.Optional(CONF_RAIN_RATE): sensor.sensor_schema(
            unit_of_measurement="mm/h", accuracy_decimals=2,
            device_class=DEVICE_CLASS_PRECIPITATION_INTENSITY, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_UV): sensor.sensor_schema(
            accuracy_decimals=0, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_LIGHT): sensor.sensor_schema(
            unit_of_measurement="W/m²", accuracy_decimals=1,
            device_class=DEVICE_CLASS_IRRADIANCE, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_SOIL): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT, accuracy_decimals=0,
            device_class=DEVICE_CLASS_MOISTURE, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_SOIL_TEMP): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS, accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_SOIL_EC): sensor.sensor_schema(
            unit_of_measurement="µS/cm", accuracy_decimals=0, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_PM25): sensor.sensor_schema(
            unit_of_measurement=UNIT_MICROGRAMS_PER_CUBIC_METER, accuracy_decimals=1,
            device_class=DEVICE_CLASS_PM25, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_PM10): sensor.sensor_schema(
            unit_of_measurement=UNIT_MICROGRAMS_PER_CUBIC_METER, accuracy_decimals=1,
            device_class=DEVICE_CLASS_PM10, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_CO2): sensor.sensor_schema(
            unit_of_measurement=UNIT_PARTS_PER_MILLION, accuracy_decimals=0,
            device_class="carbon_dioxide", state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_LEAF): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT, accuracy_decimals=0, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_LIGHTNING_STRIKES): sensor.sensor_schema(
            accuracy_decimals=0, state_class=STATE_CLASS_TOTAL_INCREASING),
        cv.Optional(CONF_LIGHTNING_DISTANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOMETER, accuracy_decimals=1,
            device_class=DEVICE_CLASS_DISTANCE, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_LEAK): sensor.sensor_schema(
            accuracy_decimals=0, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
        cv.Optional(CONF_ROSTER): text_sensor.text_sensor_schema(
            icon="mdi:access-point", entity_category="diagnostic"),
        cv.Optional(CONF_DEVICE_INFO): text_sensor.text_sensor_schema(
            icon="mdi:information-outline", entity_category="diagnostic"),
        cv.Required(CONF_I2C_ID): cv.use_id(i2c.I2CBus),
        cv.Optional(CONF_UPLOAD_SWITCH): cv.use_id(switch.Switch),
        cv.Optional(CONF_FAMILY_SWITCHES): cv.ensure_list(cv.use_id(switch.Switch)),
    }
).extend(cv.polling_component_schema("16s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    for mapping in config["remote_indoor_channels"]:
        cg.add(var.set_remote_indoor_channel(mapping["sensor_id"], mapping["channel"]))

    cg.add(var.set_device_type(config[CONF_DEVICE_TYPE]))
    cg.add(var.set_upload_host(config[CONF_ECOWITT_IP]))
    cg.add(var.set_upload_path(config[CONF_ECOWITT_PATH]))
    cg.add(var.set_upload_port(config[CONF_ECOWITT_PORT]))
    cg.add(var.set_upload_passkey(config[CONF_PASSKEY]))
    cg.add(var.set_upload_stationtype(config[CONF_STATIONTYPE]))
    cg.add(var.set_upload_freq(config[CONF_FREQ]))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE]); cg.add(var.set_temp(sens))
    if CONF_HUMIDITY in config:
        sens = await sensor.new_sensor(config[CONF_HUMIDITY]); cg.add(var.set_humidity(sens))
    if CONF_PRESSURE in config:
        sens = await sensor.new_sensor(config[CONF_PRESSURE]); cg.add(var.set_pressure(sens))
    if CONF_OUTDOOR_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_OUTDOOR_TEMPERATURE]); cg.add(var.set_outdoor_temp(sens))
    if CONF_OUTDOOR_HUMIDITY in config:
        sens = await sensor.new_sensor(config[CONF_OUTDOOR_HUMIDITY]); cg.add(var.set_outdoor_humidity(sens))
    if CONF_WIND_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_WIND_SPEED]); cg.add(var.set_wind_speed(sens))
    if CONF_WIND_DIR in config:
        sens = await sensor.new_sensor(config[CONF_WIND_DIR]); cg.add(var.set_wind_dir(sens))
    if CONF_GUST in config:
        sens = await sensor.new_sensor(config[CONF_GUST]); cg.add(var.set_gust(sens))
    if CONF_RAIN in config:
        sens = await sensor.new_sensor(config[CONF_RAIN]); cg.add(var.set_rain(sens))
    if CONF_RAIN_RATE in config:
        sens = await sensor.new_sensor(config[CONF_RAIN_RATE]); cg.add(var.set_rain_rate(sens))
    if CONF_UV in config:
        sens = await sensor.new_sensor(config[CONF_UV]); cg.add(var.set_uv(sens))
    if CONF_LIGHT in config:
        sens = await sensor.new_sensor(config[CONF_LIGHT]); cg.add(var.set_light(sens))
    if CONF_SOIL in config:
        sens = await sensor.new_sensor(config[CONF_SOIL]); cg.add(var.set_soil_moisture(sens))
    if CONF_SOIL_TEMP in config:
        sens = await sensor.new_sensor(config[CONF_SOIL_TEMP]); cg.add(var.set_soil_temp(sens))
    if CONF_SOIL_EC in config:
        sens = await sensor.new_sensor(config[CONF_SOIL_EC]); cg.add(var.set_soil_ec(sens))
    if CONF_PM25 in config:
        sens = await sensor.new_sensor(config[CONF_PM25]); cg.add(var.set_pm25(sens))
    if CONF_PM10 in config:
        sens = await sensor.new_sensor(config[CONF_PM10]); cg.add(var.set_pm10(sens))
    if CONF_CO2 in config:
        sens = await sensor.new_sensor(config[CONF_CO2]); cg.add(var.set_co2(sens))
    if CONF_LEAF in config:
        sens = await sensor.new_sensor(config[CONF_LEAF]); cg.add(var.set_leaf_wetness(sens))
    if CONF_LIGHTNING_STRIKES in config:
        sens = await sensor.new_sensor(config[CONF_LIGHTNING_STRIKES]); cg.add(var.set_lightning_strikes(sens))
    if CONF_LIGHTNING_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_LIGHTNING_DISTANCE]); cg.add(var.set_lightning_distance(sens))
    if CONF_LEAK in config:
        sens = await sensor.new_sensor(config[CONF_LEAK]); cg.add(var.set_leak(sens))

    if CONF_TIME_ID in config:
        clock = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time_clock(clock))
    if CONF_ROSTER in config:
        ts = await text_sensor.new_text_sensor(config[CONF_ROSTER])
        cg.add(var.set_roster(ts))
    if CONF_DEVICE_INFO in config:
        ts = await text_sensor.new_text_sensor(config[CONF_DEVICE_INFO])
        cg.add(var.set_device_info(ts))

    i2c_bus = await cg.get_variable(config[CONF_I2C_ID])
    cg.add(var.set_i2c_bus(i2c_bus))

    if CONF_UPLOAD_SWITCH in config:
        sw = await cg.get_variable(config[CONF_UPLOAD_SWITCH])
        cg.add(var.set_upload_switch(sw))
    if CONF_FAMILY_SWITCHES in config:
        for group, sw_id in enumerate(config[CONF_FAMILY_SWITCHES]):
            sw = await cg.get_variable(sw_id)
            cg.add(var.set_family_switch(group, sw))
