# cpx_fec

A ROS driver for the Festo CPX-FEC in remote IO mode.

The driver communicates with the CPX-FEC module over modbus.

The ROS API is not stable yet.
It currently supports configurations with up to 32 binary outputs.

## cpx_fec node

The driver is implemented as the `cpx_fec` node in the `cpx_fec` package.

### Parameters

* `~/host`

 The host or IP address to connect to. 
 
* `~/port (default: 502)`

 The port to connect to.
 
* `~/write_base (default: 40003)`

 The base register to write to.
 
* `~/read_base (default: 45395)`

 The base register to read from.

### Advertised services

* `~/set_output : cpx_fec_msgs/SetOutput`

 Set a single output. Takes an index and value, returns nothing.
 
* `~/set_outputs : cpx_fec_msgs/SetOutputs`

 Set a multiple outputs.
 Takes a 32 bit unsigned integer value representing the outputs and a 32 bit mask.
 Only bits set in the mask are affected.
 Returns nothing.
 
### Advertised topic

* `~/outputs : std_msgs/UInt32`

 The values of the outputs.
