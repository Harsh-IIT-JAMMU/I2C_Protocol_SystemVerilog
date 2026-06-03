// Generator Class
//
// The generator is responsible for creating random transactions
// and sending them to the driver through a mailbox.
//
// Think of it as a "question paper setter" that creates random
// I2C read/write requests for testing the DUT.
class generator;

  // Transaction object that stores one I2C operation
  transaction tr;

  // Mailbox used to send transactions to the driver
  mailbox #(transaction) mbxgd;

  //--------------------------------------------------
  // Events used for synchronization
  //--------------------------------------------------

  // Triggered when generator finishes sending all transactions
  event done;

  // Triggered by driver after it completes driving
  // the current transaction to the DUT
  event drvnext;

  // Triggered by scoreboard after it completes checking
  // the current transaction
  event sconext;

  // Number of transactions to generate
  int count = 0;

  //--------------------------------------------------
  // Constructor
  //--------------------------------------------------
  function new(mailbox #(transaction) mbxgd);

    // Save mailbox handle
    this.mbxgd = mbxgd;

    // Create transaction object
    tr = new();

  endfunction

  //--------------------------------------------------
  // Main Generator Task
  //--------------------------------------------------
  task run();

    // Generate 'count' number of transactions
    repeat(count) begin

      //------------------------------------------------
      // Randomize transaction fields according to
      // constraints defined in transaction class
      //------------------------------------------------
      assert(tr.randomize)
      else
        $error("Randomization Failed");

      //------------------------------------------------
      // Optional manual assignments
      // (currently commented out)
      //------------------------------------------------
      // tr.addr = 7'h12;
      // tr.din  = 8'hff;

      //------------------------------------------------
      // Send transaction to driver via mailbox
      //------------------------------------------------
      mbxgd.put(tr);

      //------------------------------------------------
      // Print generated transaction
      //------------------------------------------------
      $display("[GEN]: op :%0d, addr : %0d, din : %0d",
                tr.op, tr.addr, tr.din);

      //------------------------------------------------
      // Wait until driver finishes handling
      // this transaction
      //------------------------------------------------
      @(drvnext);

      //------------------------------------------------
      // Wait until scoreboard verifies
      // this transaction
      //------------------------------------------------
      @(sconext);

    end

    //------------------------------------------------
    // Inform environment that generation is complete
    //------------------------------------------------
    -> done;

  endtask

endclass
