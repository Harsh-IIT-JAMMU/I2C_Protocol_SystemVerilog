// Monitor Class
//
// The monitor observes DUT activity and collects
// information from the interface.
//
// Unlike the driver, the monitor NEVER drives signals.
// It only watches what is happening and records it.
//
// Think of it as a CCTV camera that observes the DUT
// and reports what happened.
class monitor;

  // Virtual interface handle
  // Used to observe DUT signals
  virtual i2c_if vif;

  // Transaction object used to store observed data
  transaction tr;

  // Mailbox used to send collected transactions
  // to the scoreboard
  mailbox #(transaction) mbxms;

  //--------------------------------------------------
  // Constructor
  //--------------------------------------------------
  function new(mailbox #(transaction) mbxms);

    // Save mailbox handle
    this.mbxms = mbxms;

  endfunction

  //--------------------------------------------------
  // Main Monitor Task
  //--------------------------------------------------
  task run();

    // Create transaction object
    tr = new();

    forever
    begin

      //----------------------------------------------
      // Wait until DUT completes a transaction
      //----------------------------------------------
      @(posedge vif.done);

      //----------------------------------------------
      // Capture all relevant interface signals
      //----------------------------------------------

      // Data sent to DUT
      tr.din  = vif.din;

      // Slave address used
      tr.addr = vif.addr;

      // Operation type
      // 0 = Write
      // 1 = Read
      tr.op   = vif.op;

      // Data returned during read operation
      tr.dout = vif.dout;

      //----------------------------------------------
      // Small delay to avoid race conditions
      //----------------------------------------------
      repeat(5) @(posedge vif.clk);

      //----------------------------------------------
      // Send collected transaction to scoreboard
      //----------------------------------------------
      mbxms.put(tr);

      //----------------------------------------------
      // Print monitored information
      //----------------------------------------------
      $display("[MON] op:%0d, addr:%0d, din:%0d, dout:%0d",
                tr.op, tr.addr, tr.din, tr.dout);

    end

  endtask

endclass
