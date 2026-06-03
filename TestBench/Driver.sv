// Driver Class
//
// The driver receives transactions from the generator
// and converts them into pin-level signals that are
// applied to the DUT (I2C Master).
//
// Think of the driver as a translator.
// Generator speaks in terms of transactions.
// DUT understands only signal values on interface pins.
class driver;

  // Virtual interface handle
  // Used to access DUT signals
  virtual i2c_if vif;

  // Transaction object received from generator
  transaction tr;

  // Event used to inform generator that
  // current transaction has been completed
  event drvnext;

  // Mailbox used to receive transactions
  mailbox #(transaction) mbxgd;

  //--------------------------------------------------
  // Constructor
  //--------------------------------------------------
  function new(mailbox #(transaction) mbxgd);
    this.mbxgd = mbxgd;
  endfunction

  //--------------------------------------------------
  // Reset Task
  //--------------------------------------------------
  // Initializes DUT signals and applies reset.
  task reset();

    // Assert reset
    vif.rst <= 1'b1;

    // Initialize all inputs
    vif.newd <= 1'b0;
    vif.op   <= 1'b0;
    vif.din  <= 0;
    vif.addr <= 0;

    // Keep reset active for 10 clock cycles
    repeat(10) @(posedge vif.clk);

    // Release reset
    vif.rst <= 1'b0;

    $display("[DRV] : RESET DONE");
    $display("---------------------------------");

  endtask

  //--------------------------------------------------
  // Write Transaction Task
  //--------------------------------------------------
  task write();

    // Ensure reset is deasserted
    vif.rst <= 1'b0;

    // Inform DUT that a new transaction is available
    vif.newd <= 1'b1;

    // Write operation
    // op = 0 => Write
    vif.op <= 1'b0;

    // Send randomized data
    vif.din <= tr.din;

    // Send randomized slave address
    vif.addr <= tr.addr;

    // Hold signals stable for few clock cycles
    repeat(5) @(posedge vif.clk);

    // Remove new transaction indication
    vif.newd <= 1'b0;

    // Wait until DUT completes operation
    @(posedge vif.done);

    $display("[DRV] : OP: WR, ADDR:%0d, DIN:%0d",
              tr.addr, tr.din);

    vif.newd <= 1'b0;

  endtask

  //--------------------------------------------------
  // Read Transaction Task
  //--------------------------------------------------
  task read();

    // Ensure reset is deasserted
    vif.rst <= 1'b0;

    // Inform DUT that a new transaction is available
    vif.newd <= 1'b1;

    // Read operation
    // op = 1 => Read
    vif.op <= 1'b1;

    // No data needs to be supplied during read
    vif.din <= 0;

    // Provide slave address to read from
    vif.addr <= tr.addr;

    // Hold request for few cycles
    repeat(5) @(posedge vif.clk);

    // Remove request indication
    vif.newd <= 1'b0;

    // Wait until DUT finishes reading
    @(posedge vif.done);

    $display("[DRV] : OP: RD, ADDR:%0d, DOUT:%0d",
              tr.addr, vif.dout);

  endtask

  //--------------------------------------------------
  // Main Driver Task
  //--------------------------------------------------
  task run();

    // Create transaction object
    tr = new();

    forever begin

      //----------------------------------------------
      // Wait until generator places a transaction
      // in mailbox
      //----------------------------------------------
      mbxgd.get(tr);

      //----------------------------------------------
      // Execute operation based on op field
      //----------------------------------------------
      if(tr.op == 1'b0)
        write();
      else
        read();

      //----------------------------------------------
      // Inform generator that driver has completed
      // current transaction
      //----------------------------------------------
      -> drvnext;

    end

  endtask

endclass
