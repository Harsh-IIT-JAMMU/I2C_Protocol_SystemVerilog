// Transaction class
// A transaction represents one complete I2C operation
// (either a read or a write request).
class transaction;

  // Indicates that a new transaction/request is available
  bit newd;

  // Operation type:
  // op = 0 -> Write operation
  // op = 1 -> Read operation
  rand bit op;

  // Data to be written to the slave device
  // Randomized by the generator
  rand bit [7:0] din;

  // 7-bit I2C slave address
  // Randomized by the generator
  rand bit [6:0] addr;

  // Data received from the slave during a read operation
  bit [7:0] dout;

  // Indicates transaction completion
  // done = 1 means operation finished
  bit done;

  // Indicates DUT is busy processing the transaction
  bit busy;

  // Acknowledge error flag
  // ack_err = 1 means slave did not acknowledge
  bit ack_err;

  //----------------------------------------------------------
  // Constraint Block 1
  //----------------------------------------------------------
  // Limits random values generated for address and data.
  //
  // Address can only be between 2 and 4.
  // Data can only be between 2 and 9.
  //
  // This keeps simulation simple and predictable.
  constraint addr_c
  {
    addr > 1;
    addr < 5;

    din > 1;
    din < 10;
  }

  //----------------------------------------------------------
  // Constraint Block 2
  //----------------------------------------------------------
  // Controls distribution of read and write operations.
  //
  // op = 0 (Write) -> 50%
  // op = 1 (Read)  -> 50%
  //
  // 'dist' is used to control probability.
  constraint rd_wr_c
  {
    op dist {1 :/ 50, 0 :/ 50};
  }

endclass
