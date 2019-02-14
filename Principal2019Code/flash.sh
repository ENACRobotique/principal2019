#!/bin/bash

scp ./Release/Principal2019Code.hex pi@eve:~/teensyBinaries/


ssh pi@eve 'teensy_loader_cli/teensy_loader_cli -mmcu=mk64fx512 -s -v -w teensyBinaries/Principal2019Code.hex'









