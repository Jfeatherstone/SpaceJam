#!/usr/bin/python3
"""
This is a script intended to be run from the command line that will solve
for all of the forces/particles given a directory of images.
"""
import numpy as np
import argparse

# My custom photoelastic toolbox
try:
    import pepe
except:
    # If it isn't installed yet, we can just add the path
    # manually
    import sys
    sys.path.append('/eno/jdfeathe/pepe/')
    #sys.path.append('/home/jack/workspaces/jupyter-workspace/pepe/')

from pepe.simulate import genSyntheticResponse
from pepe.auto import forceSolve

import matplotlib.pyplot as plt
plt.rcParams['figure.dpi'] = 140

from PIL import Image, ImageDraw

# Run parameters
#################################

rootFolder = '/eno/jdfeathe/DATA/SpaceJam/'
#rootFolder = '/run/media/jack/Seagate Portable Drive/Research/SpaceJam/'
# We'll take the data set as a argument from the command line
#dataSet = '2022-02-28_Wide'

startIndex = 40
endIndex = 400

# Our radius that we will be identifying particles with
guessRadius = 160 # [px]

# A particle is about 1.5cm across
pxPerMeter = 2*guessRadius / .015
# No idea what the actual value for this is
fSigma = 140

# How much to blur the photoelastic channel by
blurKernel = 2

# Parameters of our force solving method
contactPadding = 30
g2MaskPadding = 2
contactMaskRadius = 40
brightfield = False

cropXBounds = [200, 1200]

optimizationKwargs = {"maxEvals": [100, 150, 100], "method": 'nelder',
                       "parametersToFit": [['f'], ['f', 'a'], ['a']],
                       "allowRemoveForces": True, "useTolerance": False,
                       "allowAddForces": True, "minForceThreshold": .02,
                      "localizeAlphaOptimization": False, "imageScaleFactor": .5}


circleTrackingKwargs = {"intensitySoftmax": 2., "intensitySoftmin": 1.8, "peakDownsample": 5,
                        "offscreenParticles": False, "radiusTolerance": None, "negativeHalo": True,
                        "fitPeaks": False, "allowOverlap": True}

carryOverAlpha = True
forceNoiseWidth = .07
alphaNoiseWidth = .07

g2CalibrationCutoff = 2.

# End run parameters
#############################

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Solve for forces on a directory of images.')
    parser.add_argument('dataset', type=str, help='The name of the directory containing image files.')
    parser.add_argument('-d', type=str, help='The root output directory.', default='./')
    parser.add_argument('-e', '--ext', default='', help='The extension of the output folder.')
    parser.add_argument('-r', '--repeat', type=int, default=1, help='The number of times to repeat the solving.')
    parser.add_argument('-i', '--input_settings', type=str, help='Path to input settings file.', default=None)

    args = parser.parse_args()

    dateStr = args.dataset.split('_')[0]

    maskImage = f'./masks/{dateStr}_FullMask.bmp'
    verticalMaskImage = f'./masks/{dateStr}_VerticalMask.bmp'
    horizontalMaskImage = f'./masks/{dateStr}_HorizontalMask.bmp'

    correctionImage = rootFolder + f'calibration/{dateStr}_Calibration.bmp'
    g2CalibrationImage = rootFolder + f'calibration/{dateStr}_G2_Calibration.bmp'

    forceBalanceFactorArr = [2, 3, 4, 8, 12, 16, 20]

    for i in range(len(forceBalanceFactorArr)):

        if args.input_settings is None:
            forceArr, alphaArr, betaArr, centerArr, radiusArr, angleArr = forceSolve(rootFolder + args.dataset, guessRadius, fSigma, pxPerMeter,
                                                                brightfield, maskImage=maskImage, cropXMin=cropXBounds[0], cropXMax=cropXBounds[1],
                                                                lightCorrectionImage=correctionImage, peBlurKernel=blurKernel,
                                                                contactPadding=contactPadding, g2MaskPadding=g2MaskPadding, contactMaskRadius=contactMaskRadius,
                                                                lightCorrectionVerticalMask=verticalMaskImage, alphaNoiseWidth=alphaNoiseWidth,
                                                                lightCorrectionHorizontalMask=horizontalMaskImage, forceNoiseWidth=forceNoiseWidth,
                                                                g2CalibrationImage=g2CalibrationImage, g2CalibrationCutoffFactor=g2CalibrationCutoff,
                                                                imageStartIndex=startIndex, imageEndIndex=endIndex, carryOverAlpha=carryOverAlpha,
                                                                debug=False, optimizationKwargs=optimizationKwargs, circleTrackingKwargs=circleTrackingKwargs,
                                                                saveMovie=True, pickleArrays=True, forceBalanceWeighting=1/forceBalanceFactorArr[i],
                                                                outputRootFolder=args.d, outputExtension=args.ext + f'_1to{forceBalanceFactorArr[i]}_FBW')
            print('')
