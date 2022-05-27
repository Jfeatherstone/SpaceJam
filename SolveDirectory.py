#!/usr/bin/python3
"""
This is a script intended to be run from the command line that will solve
for all of the forces/particles given a directory of images.
"""
import numpy as np
import argparse

import multiprocessing
from setproctitle import setproctitle
import time

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

kwargs = {"imageStartIndex": None, # [frame]
          "imageEndIndex": None, # [frame]
          "guessRadius": 160, # [px]
          "fSigma": 140,
          "peBlurKernel": 2, # [px]
          "contactPadding": 30, # [px]
          "g2MaskPadding": 2, # [px]
          "contactMaskRadius": 40, # [px]
          "brightfield": False,
          "cropXMin": 200, # [px]
          "cropXMax": 1200, # [px]
          "carryOverForce": False,
          "carryOverAlpha": False,
          "forceNoiseWidth": .03,
          "alphaNoiseWidth": .03,
          "g2CalibrationCutoffFactor": 2,
          "performOptimization": False, # For debug purposes
         }

# Particle is approx. 15 mm across
kwargs["pxPerMeter"] = 2 * kwargs["guessRadius"] / 0.15

optimizationKwargs = {"maxEvals": [150, 300, 150], "method": 'nelder',
                      "parametersToFit": [['f'], ['f', 'a'], ['a']],
                      "allowRemoveForces": True, "useTolerance": False,
                      "allowAddForces": True, "minForceThreshold": .02,
                      "localizeAlphaOptimization": False, "imageScaleFactor": 2,
                      "forceBalanceWeighting": .03}


circleTrackingKwargs = {"intensitySoftmax": 2., "intensitySoftmin": 1.8, "peakDownsample": 5,
                        "offscreenParticles": False, "radiusTolerance": None, "negativeHalo": True,
                        "fitPeaks": False, "allowOverlap": True}

# End run parameters
#############################

#############################
# Threading

workQueueLock = multiprocessing.Lock()
completedCountLock = multiprocessing.Lock()
completedCount = 0

# A simple struct to store macro information about each solve operation.
# This is more specific than just specifying the dataset name eg. if we
# are repeating each solving 3 times, we will have three of these structs.
class DataStruct():

    def __init__(self, datasetName, extension='', outputFolder='./', repeatNum=None, inputSettings=None, lineNum=0):
        self.datasetName = datasetName
        self.repeatNum = repeatNum
        self.extension = extension
        self.outputFolder = outputFolder
        self.inputSettings = inputSettings
        self.lineNum = lineNum

        # Generate the trial specific things like masks and correction images.
        # These are trial specific because they can be read from the date.
        dateStr = datasetName.split('_')[0]

        self.maskImage = f'./masks/{dateStr}_FullMask.bmp'
        self.verticalMaskImage = f'./masks/{dateStr}_VerticalMask.bmp'
        self.horizontalMaskImage = f'./masks/{dateStr}_HorizontalMask.bmp'

        self.correctionImage = rootFolder + f'calibration/{dateStr}_Calibration.bmp'
        self.g2CalibrationImage = rootFolder + f'calibration/{dateStr}_G2_Calibration.bmp'

    def __str__(self):
        return self.datasetName + (self.repeatNum if self.repeatNum is not None else '')


class SolveProcessor(multiprocessing.Process):

    def __init__(self, processorName, workQueue):
        # Call super
        multiprocessing.Process.__init__(self, target=self.run)

        self.processorName = processorName
        self.workQueue = workQueue

        self.active = True

    def run(self):
        global completedCount
        setproctitle(self.processorName)

        while self.active:
            # Will hang until the queue in available
            workQueueLock.acquire()
            # Make sure we have something to process
            if not self.workQueue.empty():

                # Grab the next data set to process
                dataset = self.workQueue.get()
                # Release the lock so the next thread can find its dataset
                workQueueLock.release()
                
                # Check how many items have been processed, so we can determine
                # which line to put the progress bar on
                completedCountLock.acquire()
                progressBarOffset = dataset.lineNum - completedCount
                completedCountLock.release()

                # Now we actually process the dataset
                #startTime = time.perf_counter()
                # Can't have a progress bar because the multiple threads will make a mess
                # of stdout
                solveDataset(dataset, progressBarOffset, self.processorName)

                completedCountLock.acquire()
                completedCount += 1
                completedCountLock.release()
                #dt = int(time.perf_counter() - startTime)


            else:
                # If there is no more work, we release the queue lock and exit on the next loop iteration.
                # This shouldn't cause any issues because we load all of our data at the beginning,
                # and don't get any more afterwards.
                workQueueLock.release()
                self.active = False

        # When the thread no longer has anything to do, we join it back to the main one
        #self.join()

def solveDataset(dataStruct, progressBarOffset=0, processorName=None):

    if dataStruct.inputSettings is None:
        forceArr, alphaArr, betaArr, centerArr, radiusArr, angleArr = forceSolve(rootFolder + dataStruct.datasetName, **kwargs,
                                                            showProgressBar=True, maskImage=dataStruct.maskImage,
                                                            lightCorrectionImage=dataStruct.correctionImage,
                                                            lightCorrectionVerticalMask=dataStruct.verticalMaskImage,
                                                            lightCorrectionHorizontalMask=dataStruct.horizontalMaskImage,
                                                            g2CalibrationImage=dataStruct.g2CalibrationImage,
                                                            debug=False, optimizationKwargs=optimizationKwargs, circleTrackingKwargs=circleTrackingKwargs,
                                                            saveMovie=True, pickleArrays=True, outputRootFolder=dataStruct.outputFolder,
                                                            progressBarOffset=progressBarOffset, progressBarTitle=str(dataStruct) + (f' ({processorName})' if processorName is not None else ''),
                                                            outputExtension=dataStruct.extension + (f'_{dataStruct.repeatNum}' if dataStruct.repeatNum is not None else ''))

    else:
        forceArr, alphaArr, betaArr, centerArr, radiusArr, angleArr = forceSolve(rootFolder + dataStruct.datasetName, inputSettingsFile=dataStruct.inputSettings,
                                                            maskImage=dataStruct.maskImage, g2CalibrationImage=dataStruct.g2CalibrationImage,
                                                            lightCorrectionImage=dataStruct.correctionImage, showProgressBar=True,
                                                            lightCorrectionVerticalMask=dataStruct.verticalMaskImage,
                                                            lightCorrectionHorizontalMask=dataStruct.horizontalMaskImage,
                                                            progressBarOffset=progressBarOffset, progressBarTitle=str(dataStruct) + (f' ({processorName})' if processorName is not None else ''),
                                                            outputExtension=dataStruct.extension + (f'_{dataStruct.repeatNum}' if dataStruct.repeatNum is not None else ''))

# End threading
#############################

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Solve for forces on a directory of images.')
    parser.add_argument('dataset', type=str, help='The name of the directory containing image files.', nargs='+')
    parser.add_argument('-d', type=str, help='The root output directory.', default='./')
    parser.add_argument('-e', '--ext', default='', help='The extension of the output folder.')
    parser.add_argument('-r', '--repeat', type=int, default=1, help='The number of times to repeat the solving.')
    parser.add_argument('-i', '--input_settings', type=str, help='Path to input settings file.', default=None)
    parser.add_argument('-p', '--processors', type=int, default=1, help='The number of processors to use.')

    args = parser.parse_args()

    workQueueLock.acquire()

    # Populate our work queue
    workQueue = multiprocessing.Queue()
    totalItems = int(len(args.dataset)*args.repeat)

    print(f'Solving {totalItems} item(s):')
    for i in range(len(args.dataset)):
        for j in range(args.repeat):
            # Create the data struct to pass to the threads (even if we only have one)
            dataStruct = DataStruct(args.dataset[i], extension=args.ext, outputFolder=args.d, repeatNum=j if args.repeat > 1 else None, inputSettings=args.input_settings, lineNum=i*args.repeat + j)
            workQueue.put(dataStruct)
            print(5*' ' + str(dataStruct))

    workQueueLock.release()

    # If the provided number of threads is less than zero, that means
    # use as up to the total number of cores, minus that number.
    # eg. on a 24 core system, -14 will use up to 10 cores.
    numProcessors = args.processors if args.processors > 0 else multiprocessing.cpu_count() + args.processors
    # We don't want more cores than we have tasks, so we cap the number of
    # cores based on how many systems we are solving
    numProcessors = min(numProcessors, totalItems)

    # If only a single thread, we just solve as normal
    if numProcessors == 1:
        print(f'Utilizing 1 processor.\n')
        while not workQueue.empty():
            print('solving')
            dataset = workQueue.get()

            solveDataset(dataset)

    else:
        print(f'Utilizing {numProcessors} + 1 processors.\n')

        # Otherwise, we generate our processes
        processorArr = []
        for i in range(numProcessors):
            processor = SolveProcessor(f'space_jam_{i}', workQueue)
            processor.start()
            processorArr.append(processor)

        # Now wait until all of the threads finish. They will automatically
        # close when there is no longer any work to do, so we just have
        # to monitor the number active threads. The main thread counts as 1,
        # so when the number of threads is equal to 1, that means they have all
        # finished.
        for i in range(numProcessors):
            processorArr[i].join()

        #while np.sum(np.int64([p.is_alive() for p in processorArr])) > 0:
        #    # Sleep 10 seconds in between each check
        #    time.sleep(5)

    print('')
