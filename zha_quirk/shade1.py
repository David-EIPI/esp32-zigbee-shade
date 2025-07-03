from zigpy.quirks.v2 import QuirkBuilder, ReportingConfig
from zigpy.zcl.clusters.closures import WindowCovering
from zigpy.quirks.v2.homeassistant.sensor import SensorDeviceClass, SensorStateClass
from zha.application.platforms.number.const import NumberMode
from zigpy.quirks.v2.homeassistant.number import NumberDeviceClass
from zigpy.quirks.v2 import CustomDeviceV2


DS2_QUIRK_ID = "ds_shade1_quirk"


# Configurable lift limits, in cm. These do not have to be accurate to operate the shade,
# but if set correctly value of the CurrentPosition attribute will also be meaningful.
UPPER_LIFT_LIMIT=180
LOWER_LIFT_LIMIT=0

#
# Apply the quirk
#
class ds2QuirkDevice(CustomDeviceV2):
    quirk_id = DS2_QUIRK_ID
(
    QuirkBuilder("DS", "Shade1")
    .device_class(ds2QuirkDevice)

    .write_attr_button(
        attribute_name = "installed_open_limit_lift",
        attribute_value = UPPER_LIFT_LIMIT,
        cluster_id = WindowCovering.cluster_id,
        endpoint_id = 1,
        fallback_name = "Set upper limit",
        translation_key = "upper_limit",
    )
    .write_attr_button(
        attribute_name = "installed_closed_limit_lift",
        attribute_value = LOWER_LIFT_LIMIT,
        cluster_id = WindowCovering.cluster_id,
        endpoint_id = 1,
        fallback_name = "Set lower limit",
        translation_key = "lower_limit",
    )
    .write_attr_button(
        attribute_name = "window_covering_mode",
        attribute_value = 2,
        cluster_id = WindowCovering.cluster_id,
        endpoint_id = 1,
        fallback_name = "Reset limits",
        translation_key = "reset",
    )
    .number(
        attribute_name = "velocity_lift",
        cluster_id = WindowCovering.cluster_id,
        endpoint_id = 1,
        fallback_name = "Lift velocity",
        translation_key = "velocity",
        device_class = NumberDeviceClass.SPEED,
        unit = "cm/s",
        mode = NumberMode.SLIDER,
        min_value = 1,
        max_value = 10,
    )
    .sensor(
        attribute_name = "current_position_lift_percentage",
        cluster_id = WindowCovering.cluster_id,
        endpoint_id = 1,
        reporting_config=ReportingConfig(
            min_interval=10, max_interval=3600, reportable_change=1
        ),
        fallback_name = "Coverage",
        translation_key = "coverage",
        state_class = SensorStateClass.MEASUREMENT,
        unit = "%",
    )
    .add_to_registry()
)
