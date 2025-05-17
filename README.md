# carla_examples
This repository the scripts that can spawn vehicles , which have the specific state ( location, velocity, autopilot ...)

## .py files 
```
batchProcssing & batchProcessing2 : the main proceessing file of scenario1&2
batchTest & batchTest2 : the auto script to run the scenario1&2
```

```
multi_scenario : the main proceessing file of scenario 3\4\5\6 demo, not a batch proess
```

```
generateHardEncodeParamScenario_1\ 2\ : the auto script to run the scenario 1\2  demo, and create 400 .py file based on the batchProcessing.py and batchProcessing2_3cars.py, each of them is a single processing file with the different hard encode parameters

generateHardEncodeParamScenario_3\ 4\ 5\ 6 : the auto script to run the scenario 3\4\5\6 demo, and create 400 .py file based on the multi_scenario.py, each of them is a single processing file with the different hard encode parameters
```

## update logs
### 2025.05.13-15
1. add the batch processing script for scenario 1~6
2. add the auto script to run the scenario 1~6 demo, and create 400 .py file based on the batchProcessing.py\batchProcessing2_3cars.py  and multi_scenario.py, each of them is a single processing file with the different hard encode parameters

### 2024.12.05
1. add the scenario 1 and its batch processing script

### 2024.12.04
1. add the batch processing script for scenario 4 
2. attention : it just change on the origin code, and you just need to open the batch process mode in the argparse

### 2024.12.03
1. use Traffic Manager to set up background vehicles, for a single processing 
2. batch processing the scenario 1&2 , and record the demo video 

### 2024.12.02
1. add the scenario 3~6 demo
2. add the batch processing script for scenario 1&2

