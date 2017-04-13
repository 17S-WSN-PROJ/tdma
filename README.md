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


## Slip Server
   just run sh server.sh in safebike/data_collection/driver_accelerometers
   You should change the address of the host firefly in server.sh, if you’re doing it with a mac just find it with `ls/dev/tty.*` 
   

## Slip Client
   just run sh driver.sh in safebike/data_collection/driver_accelerometers after you’ve started the server. I kept them in separate scripts because it’s useful to see both their logs. 
   if you’re seeing a v=20 being printed it’s working.
   output is in test.csv, you can also run it with -v to make it list the definition of every printed thing. 
   output format of every line:
	timestamp,microseconds,sequence,mac,accX,accY,accZ,gyrX,gyrY,gyrZ,hmcX,hmcY,hmcZ

For your own sake, just always use Ctrl-C over Ctrl-Z when ending the slip client to make sure everything gets written. Also the kernel freaks out if you Ctrl-Z out of the server. 

