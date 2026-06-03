// Scoreboard Class
//
// The scoreboard acts as the "checker" of the testbench.
//
// It receives transactions from the monitor and compares
// DUT results against expected results.
//
// Think of it as an examiner checking whether a student's
// answer matches the correct answer.
class scoreboard;

  // Transaction received from monitor
  transaction tr;

  // Mailbox used to receive monitored transactions
  mailbox #(transaction) mbxms;

  // Event used to inform generator that
  // scoreboard has finished checking current transaction
  event sconext;

  //--------------------------------------------------
  // Reference Model Storage
  //--------------------------------------------------

  // Temporary variable used during read checking
  bit [7:0] temp;

  // Reference memory model
  //
  // This memory represents what the DUT memory
  // SHOULD contain if it is functioning correctly.
  //
  // Address range = 0 to 127
  bit [7:0] mem[128] = '{default:0};

  //--------------------------------------------------
  // Constructor
  //--------------------------------------------------
  function new(mailbox #(transaction) mbxms);

    // Save mailbox handle
    this.mbxms = mbxms;

    //------------------------------------------------
    // Initialize reference memory
    //------------------------------------------------
    //
    // mem[0] = 0
    // mem[1] = 1
    // mem[2] = 2
    // ...
    // mem[127] = 127
    //
    // This gives predictable initial contents.
    //------------------------------------------------
    for(int i = 0; i < 128; i++)
    begin
      mem[i] <= i;
    end

  endfunction

  //--------------------------------------------------
  // Main Scoreboard Task
  //--------------------------------------------------
  task run();

    forever begin

      //----------------------------------------------
      // Wait for monitor to send a transaction
      //----------------------------------------------
      mbxms.get(tr);

      //----------------------------------------------
      // Save expected value currently present
      // at the requested address
      //----------------------------------------------
      temp = mem[tr.addr];

      //----------------------------------------------
      // WRITE Operation
      //----------------------------------------------
      if(tr.op == 1'b0)
      begin

        // Update reference model
        mem[tr.addr] = tr.din;

        $display("[SCO]: DATA STORED -> ADDR:%0d DATA:%0d",
                  tr.addr, tr.din);

        $display("-----------------------------------------------");

      end

      //----------------------------------------------
      // READ Operation
      //----------------------------------------------
      else
      begin

        //--------------------------------------------
        // Compare DUT output against expected value
        //--------------------------------------------
        if(tr.dout == temp)
          $display("[SCO]: DATA READ -> DATA MATCHED exp:%0d rec:%0d",
                    temp, tr.dout);
        else
          $display("[SCO]: DATA READ -> DATA MISMATCHED exp:%0d rec:%0d",
                    temp, tr.dout);

        $display("-----------------------------------------------");

      end

      //----------------------------------------------
      // Inform generator that scoreboard
      // has completed checking
      //----------------------------------------------
      -> sconext;

    end

  endtask

endclass
