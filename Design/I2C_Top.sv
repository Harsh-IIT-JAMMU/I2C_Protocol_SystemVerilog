`timescale 1ns / 1ps
 
// Top module that connects I2C Master and Slave together
module i2c_top(
    input clk, rst,        // clk = system clock, rst = reset signal
    input newd,            // newd = tells master "new data is ready"
    input op,              // op = operation (0 = write, 1 = read)
    input [6:0] addr,      // 7-bit I2C device address
    input [7:0] din,       // data to be sent from master to slave
    output [7:0] dout,     // data received from slave to master
    output busy,           // indicates master is busy doing a transaction
    output ack_err,        // indicates if any ACK error happened
    output done            // indicates transaction is completed
);

// These are the two I2C communication lines
// SDA = data line, SCL = clock line
wire sda, scl;

// Separate error signals from master and slave
wire ack_errm, ack_errs;
 
 
// Instantiate I2C Master
// Master controls communication (start, stop, read, write)
i2c_master master (
    clk, rst, newd, addr, op,
    sda, scl,         // shared I2C lines
    din, dout,        // input/output data
    busy,             // tells if master is working
    ack_errm,         // ACK error from master side
    done              // transaction complete
);

// Instantiate I2C Slave
// Slave responds when master talks to it
i2c_Slave slave (
    scl, clk, rst,
    sda,              // shared data line
    ack_errs          // ACK error from slave side
);
 
// Final ACK error = if either master OR slave reports error
assign ack_err = ack_errs | ack_errm;
 
endmodule
