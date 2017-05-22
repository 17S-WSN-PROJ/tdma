##Overview

This code is intended for use in a gait monitoring system where the client nodes control
Razor 9 DoF IMUs to gather acceleration and orientation data that is relayed to the
master. The code is adapted from the Safebike project at Carnegie Mellon University
(wise.ece.cmu.edu/redmine/projects/safebike/wiki). 

Once the nodes sync to the master's time beacon, they each send data in their designated
time slots. The master node parses the data and outputs it over serial to your favorite
terminal. 

## Programming Nodes

Program them with what’s in safebike/firefly/safe_cyclist/tdma_client

```
make clean
make CLIENT_MAC=1 program 
# CLIENT_MAC here should be distinct numbers above 0. So you’ll probably use 1 2 3 4…
```

## Programming Host

Program them with what’s in safebike/firefly/safe_cyclist/tdma_master
```
make clean
make program
```



