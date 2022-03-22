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

startIndex = None
endIndex = None

# Our radius that we will be identifying particles with
guessRadius = 160 # [px]

# A particle is about 1cm across
pxPerMeter = 2*guessRadius / .01
# No idea what the actual value for this is
fSigma = 140

# How much to blur the photoelastic channel by
blurKernel = 2

# Parameters of our force solving method
contactPadding = 20
g2MaskPadding = 2
contactMaskRadius = 40
brightfield = False

maskImage = './Masks/2022-03-16_FullMask.bmp'
verticalMaskImage = './Masks/2022-03-16_VerticalMask.bmp'
horizontalMaskImage = './Masks/2022-03-16_HorizontalMask.bmp'

correctionImage = rootFolder + 'calibration/2022-03-16_Calibration.bmp'
g2CalibrationImage = rootFolder + 'calibration/2022-03-16_G2_Calibration.bmp'

optimizationKwargs = {"maxEvals": [120, 100], "method": 'nelder',
                     "parametersToFit": [['f'], ['a']],
                     "allowRemoveForces": False, "alphaTolerance": 2., "forceTolerance": 1.,
                     "allowAddForces": False, "minForceThreshold": .03,
                      "localizeAlphaOptimization": True, "imageScaleFactor": 0.5}

carryOverAlpha = True
forceNoiseWidth = .1

g2CalibrationCutoff = 1.8

# End run parameters
#############################

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Solve for forces on a directory of images.')
    parser.add_argument('dataset', type=str, help='The name of the directory containing image files.')
    parser.add_argument('output_dir', type=str, help='The root output directory.', default='./')
    parser.add_argument('-e', '--ext', default='', help='The extension of the output folder.')

    args = parser.parse_args()

    forceArr, alphaArr, betaArr, centerArr, radiusArr = forceSolve(rootFolder + args.dataset, guessRadius, fSigma, pxPerMeter,
                                                            brightfield, maskImage=maskImage, lightCorrectionImage=correctionImage,
                                                            lightCorrectionVerticalMask=verticalMaskImage,
                                                            lightCorrectionHorizontalMask=horizontalMaskImage, forceNoiseWidth=forceNoiseWidth,
                                                            g2CalibrationImage=g2CalibrationImage, g2CalibrationCutoffFactor=g2CalibrationCutoff,
                                                            imageStartIndex=startIndex, imageEndIndex=endIndex, carryOverAlpha=carryOverAlpha,
                                                            debug=False, optimizationKwargs=optimizationKwargs, saveMovie=True, pickleArrays=True,
                                                            outputRootFolder=args.output_dir, outputExtension=args.ext)
