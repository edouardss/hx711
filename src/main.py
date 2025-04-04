import asyncio
from typing import Any, ClassVar, Dict, Final, Mapping, Optional, Sequence

from typing_extensions import Self
from viam.components.sensor import *
from viam.module.module import Module
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import SensorReading, struct_to_dict, ValueTypes
import random
from hx711 import HX711
import RPi.GPIO as GPIO

class Loadcell(Sensor, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("edss", "hx711-loadcell"), "loadcell")

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        return super().new(config, dependencies)

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        fields = config.attributes.fields

        if "gain" in fields:
            if not fields["gain"].HasField("number_value"):
                raise Exception("Gain must be a valid number.")
        if "doutPin" in fields:
            if not fields["doutPin"].HasField("number_value"):
                raise Exception("Data Out pin must be a valid number.")
        if "sckPin" in fields:
            if not fields["sckPin"].HasField("number_value"):
                raise Exception("Gain must be a valid number.")
        if "numberOfReadings" in fields:
            if not fields["numberOfReadings"].HasField("number_value"):
                raise Exception("Gain must be a valid number.")
        if "tare_offset" in fields:
            if not fields["tare_offset"].HasField("number_value"):
                raise Exception("Tare offset must be a valid number.")
        # If all checks pass, return an empty list indicating no errors

        return []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        attrs = struct_to_dict(config.attributes)
        self.gain = float(attrs.get("gain", 64))
        self.doutPin = int(attrs.get("doutPin", 5))
        self.sckPin = int(attrs.get("sckPin", 6))
        self.numberOfReadings = int(attrs.get("numberOfReadings", 3))
        self.tare_offset = float(attrs.get("tare_offset", 0.0))
        return super().reconfigure(config, dependencies)

    async def get_readings(
        self,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, SensorReading]:
        
        try:
            hx711 = HX711(
                dout_pin=self.doutPin,
                pd_sck_pin=self.sckPin,
                channel='A',
                gain=self.gain
            )

            hx711.reset()   # Reset the HX711 before starting
            measures = hx711.get_raw_data(times=self.numberOfReadings)
            avg = sum(measures) / len(measures)
            avg -= self.tare_offset  # Subtract tare offset from readings
        finally:
            GPIO.cleanup()  # Always clean up GPIO

        # Return a dictionary of the readings
        return {
            "weight": avg
        }

    async def tare(self):
        """Tare the load cell by setting the current reading as the zero offset."""
        
        try:
            hx711 = HX711(
                dout_pin=self.doutPin,
                pd_sck_pin=self.sckPin,
                channel='A',
                gain=self.gain
            )

            hx711.reset()   # Reset HX711 before starting
            measures = hx711.get_raw_data(times=self.numberOfReadings)
            self.tare_offset = sum(measures) / len(measures)  # Set tare offset
        finally:
            GPIO.cleanup()  # Always clean up GPIO

async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, ValueTypes]:
        result = {key: False for key in command.keys()}
        for (name, args) in command.items():
            if name == 'tare':
                self.tare(*args)
                result[name] = True
        return result

if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
