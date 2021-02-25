#!/usr/bin/env python3
"""
IOC for simulating one or more sets of tip-tilt mirrors paired with a pair of
cameras for controlling NF/FF pointing of a laser beam on a camera. 

Camera pointing is reported using a centroid.

Tip-tilt stages are "open-loop", controlled using step PVs. 
"""
import time
import numpy as np

#from caproto.ioc_examples.mini_beamline import MovingDot

from caproto.server import PVGroup, SubGroup, ioc_arg_parser, pvproperty, run

class SimCameraCentroid(PVGroup):
    """
    A fake camera that reports X and Y centroid positions, with noise, based on
    a control input and a calibration factor.
    """
    def __init__(self, *args, width, height, **kwargs):
        self.width = width
        self.height = height
        super().__init__(*args, **kwargs)

    Centroid_X = pvproperty(value=0.0, record='ao', read_only=True)
    Centroid_Y = pvproperty(value=0.0, record='ao', read_only=True)

    # Control knob to apply noise to X/Y centroid positions
    NOISE = pvproperty(value=5.0, record='ai')

    # Control knobs for X/Y centroid positions. Used to couple external motion
    # to centroid position.
    NOMINAL_X = pvproperty(value=0, record='ai')
    NOMINAL_Y = pvproperty(value=0, record='ai')

    # Factors to relate external motor position to shift of centroid
    XFACTOR = pvproperty(value=1.0, record='ai')
    YFACTOR = pvproperty(value=1.0, record='ai')

    @NOMINAL_X.startup
    async def NOMINAL_X(self, instance, async_lib):
        await instance.write(value=self.width/2)

    @NOMINAL_Y.startup
    async def NOMINAL_Y(self, instance, async_lib):
        await instance.write(value=self.height/2)

    @Centroid_X.scan(period=0.2, use_scan_field=True)
    async def Centroid_X(self, instance, async_lib):
        if hasattr(self, 'xcurrent'):
            # TODO: Deal with values outside FOV
            pixel_noise = np.random.random_sample()*(2*self.NOISE.value - 1.0)
            self.xcurrent = self.NOMINAL_X.value + pixel_noise
            await instance.write(self.xcurrent)
        else:  # Default to middle of camera
            self.xcurrent = self.width/2
            await instance.write(self.xcurrent)
    
    @Centroid_Y.scan(period=0.2, use_scan_field=True)
    async def Centroid_Y(self, instance, async_lib):
        if hasattr(self, 'ycurrent'):
            # TODO: Deal with values outside FOV
            pixel_noise = np.random.random_sample()*(2*self.NOISE.value - 1.0)
            self.ycurrent = self.NOMINAL_Y.value + pixel_noise
            await instance.write(self.ycurrent)
        else:  # Default to middle of camera
            self.ycurrent = self.height/2
            await instance.write(self.ycurrent)

class SimMotor(PVGroup):
    STEPS = pvproperty(value=0, record='longin', doc='Integer steps')
    VOLTAGE = pvproperty(value=0, record='longin', doc='16 bit voltage')
   

class MovingDot(PVGroup):
    N = 480
    M = 640

    sigmax = 50
    sigmay = 25

    background = 1000

    Xcen = Ycen = 0

    ArrayData = pvproperty(value=[0] * N * M,
                     dtype=float,
                     read_only=True,
                     doc=f'Detector image ({N}x{M})'
                     )

    @ArrayData.getter
    async def ArrayData(self, instance):
        N = self.N
        M = self.M
        back = np.random.poisson(self.background, (N, M))
        if not self.shutter_open.value:
            await self.img_sum.write([back.sum()])
            return back.ravel()
        x = self.mtrx.value
        y = self.mtry.value

        Y, X = np.ogrid[:N, :M]

        X = X - M / 2 + x
        Y = Y - N / 2 + y

        X /= self.sigmax
        Y /= self.sigmay

        dot = np.exp(-(X**2 + Y**2) / 2) * np.exp(- (x**2 + y**2) / 100**2)

        I = self.parent.current.value  # noqa
        e = self.exp.value
        measured = (self.parent.N_per_I_per_s * dot * e * I)
        ret = (back + np.random.poisson(measured))
        await self.img_sum.write([ret.sum()])
        return ret.ravel()

    img_sum = pvproperty(value=0, read_only=True, dtype=float)
    mtrx = pvproperty(value=0, dtype=float)
    mtry = pvproperty(value=0, dtype=float)

    exp = pvproperty(value=1, dtype=float)

    @exp.putter
    async def exp(self, instance, value):
        value = np.clip(value, a_min=0, a_max=None)
        return value

    shutter_open = pvproperty(value=1, dtype=int, doc='Shutter open/close')

    ArraySizeY_RBV = pvproperty(value=N, dtype=int,
                                read_only=True, doc='Image array size Y')
    ArraySizeX_RBV = pvproperty(value=M, dtype=int,
                                read_only=True, doc='Image array size X')
    ArraySize_RBV = pvproperty(value=[N, M], dtype=int,
                               read_only=True, doc='Image array size [Y, X]')

class PointingSimulator(PVGroup):
    """
    Collection of simulated cameras and motors making up a pointing assembly.
    """
#    # Control knobs for relating motor steps to camera image motion 
#    FF_XFACTOR = pvproperty(value=1.0, record='ai')
#    FF_YFACTOR = pvproperty(value=1.0, record='ai')
#
#    # Cameras
#    NF = SubGroup(SimCameraCentroid, width=2592, height=1944, doc='NF Camera')
#    FF = SubGroup(SimCameraCentroid, width=1440, height=1080, doc='FF Camera')
#
#    # Motors
#    TIP = SubGroup(SimMotor)
#    TILT = SubGroup(SimMotor)

    N_per_I_per_s = 200

    current = pvproperty(value=500, dtype=float, read_only=True)

    @current.scan(period=0.1)
    async def current(self, instance, async_lib):
        current = 500 + 25 * np.sin(time.monotonic() * (2 * np.pi) / 4)
        await instance.write(value=current)

    NF = SubGroup(MovingDot)

if __name__ == '__main__':
    ioc_options, run_options = ioc_arg_parser(
        default_prefix='LAS:TEST:',
        desc=('An IOC that simulates a pair of tip-tilt mirrors and a NF/FF '
              'camera pair used for laser pointing/centering.'))

    print(ioc_options)
    print(run_options)
    ioc = PointingSimulator(**ioc_options)
    run(ioc.pvdb, **run_options)
