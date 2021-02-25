#!/usr/bin/env python3
"""
IOC for simulating one or more sets of tip-tilt mirrors paired with a pair of
cameras for controlling NF/FF pointing of a laser beam on a camera. 

Camera pointing is reported using a centroid.

Tip-tilt stages are "open-loop", controlled using step PVs. 
"""
import time
import numpy as np

from caproto.server import PVGroup, SubGroup, ioc_arg_parser, pvproperty, run

class SimPointingCamera(PVGroup):
    """
    A fake camera that reports X and Y centroid positions, with noise, based on
    a coupled tip-tilt motor and a calibration factor.
    """
    def __init__(self, *args, width, height, **kwargs):
        self.width = width
        self.height = height
        super().__init__(*args, **kwargs)

    Centroid_X = pvproperty(value=0.0, record='ao', read_only=True)
    Centroid_Y = pvproperty(value=0.0, record='ao', read_only=True)

    # Control knob to apply noise to X/Y centroid positions
    NOISE = pvproperty(value=5.0, record='ai')

    # Factors to relate external motor position to shift of centroid
    XFACTOR = pvproperty(value=1.0, record='ai')
    YFACTOR = pvproperty(value=1.0, record='ai')

    NOMINAL_X = pvproperty(value=0, record='ai')
    NOMINAL_Y = pvproperty(value=0, record='ai')

    @NOMINAL_X.startup
    async def NOMINAL_X(self, instance, async_lib):
        await instance.write(self.width/2)

    @NOMINAL_Y.startup
    async def NOMINAL_Y(self, instance, async_lib):
        await instance.write(self.height/2)

    TIP_STEPS = pvproperty(value=0, record='longin', doc='Integer steps')
    TIP_VOLTAGE = pvproperty(value=0, record='longin', doc='16 bit voltage')

    TILT_STEPS = pvproperty(value=0, record='longin', doc='Integer steps')
    TILT_VOLTAGE = pvproperty(value=0, record='longin', doc='16 bit voltage')
   
    @Centroid_X.scan(period=0.2, use_scan_field=True)
    async def Centroid_X(self, instance, async_lib):
        if hasattr(self, 'xcurrent'):
            # TODO: Deal with values outside FOV
            pixel_noise = np.random.random_sample()*(2*self.NOISE.value - 1.0)
            self.xcurrent = self.NOMINAL_X.value + self.TILT_STEPS.value*self.XFACTOR.value + pixel_noise
            await instance.write(self.xcurrent)
        else:  # Default to middle of camera at startup
            self.xcurrent = self.width/2
            await instance.write(self.xcurrent)
    
    @Centroid_Y.scan(period=0.2, use_scan_field=True)
    async def Centroid_Y(self, instance, async_lib):
        if hasattr(self, 'ycurrent'):
            # TODO: Deal with values outside FOV
            pixel_noise = np.random.random_sample()*(2*self.NOISE.value - 1.0)
            self.ycurrent = self.NOMINAL_Y.value + self.TIP_STEPS.value*self.YFACTOR.value + pixel_noise
            await instance.write(self.ycurrent)
        else:  # Default to middle of camera at startup
            self.ycurrent = self.height/2
            await instance.write(self.ycurrent)

class PointingSimulator(PVGroup):
    """
    Collection of simulated cameras and motors making up a pointing assembly.
    """
    # Cameras
    NF = SubGroup(SimPointingCamera, width=2592, height=1944, doc='NF Camera')
    FF = SubGroup(SimPointingCamera, width=1440, height=1080, doc='FF Camera')

if __name__ == '__main__':
    ioc_options, run_options = ioc_arg_parser(
        default_prefix='LAS:TEST:',
        desc=('An IOC that simulates a pair of tip-tilt mirrors and a NF/FF '
              'camera pair used for laser pointing/centering.'))

    print(ioc_options)
    print(run_options)
    ioc = PointingSimulator(**ioc_options)
    run(ioc.pvdb, **run_options)
