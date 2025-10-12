# new-brain-app

## Project Overview
This application aims to filter EEG data in real time using signal processing techniques and map the amplitude of their output to LED lights.

## Pipeline
1. Input __Batch__
2. Set initial reference
3. Band-pass filter
4. Notch filter
5. _Artifact Removal_
6. Common average regerence (CAR)
7. Min-max normalization
8. _Fast Fourier transform (FFT)_
7. Output __Batches__

### Initial Steps
1. Write new batches of EEG data to .json files stored in a folder. Each batch should contain 512 timepoints with EEG data being recorded at 256 Hz. Data should be formated like the example below:

{
  "batch_512": {
    "f8": [12.4, 13.1, 12.9, 14.2, 13.7, 12.8, ... ],
    "cz": [8.7, 9.1, 8.9, 9.4, 9.0, 8.8, ... ],
    "a1": [10.3, 10.5, 10.7, 10.2, 10.6, 10.4, ... ],
  }
}

Note: 'a1' & 'a2' or 'cz' will be the reference electrodes.

2. Create an EEGBatch class that can load in a single batch of data and be used to pass to the filtering functions. Within the class we will configure how to organize the channels and remove the channel names.
3. Set the initial reference using the reference electrodes.
4. Band-pass filter (1 Hz - 40 Hz recommended for now). This is the high-pass and low-pass combined.
5. Notch filter (@ 60 hz).
6. Common average reference (CAR): subtract the mean across all 16 electrodes.
7. Min-max normalization to make sure all the waves have the same amplitude range.
8. Output the processed batch into .json file in a new folder.

Notes: 
- We will not be doing baselining or artifact removal in this pipeline.
- We can cut the sample-data.json into batches and feed them into the pipeline to simulate the process during development.
- Artifact removal will occur between before CAR and will be handled by a machine learning model.

Optional:
- We can apply Fast Fourier transform (FFT) after min-max normalization and store multiple batches for each frequency bin. This would allow us to display different frequencies on the dress in diffrent colors (e.g., delta, gamma, beta, theta, ...).

