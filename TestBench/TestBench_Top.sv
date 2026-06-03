//======================================================
// Top-Level Testbench
//======================================================
//
// This module creates and connects all verification
// components:
//
// Generator  -> Creates transactions
// Driver     -> Drives transactions to DUT
// Monitor    -> Observes DUT behavior
// Scoreboard -> Checks correctness
//
// It also creates:
// - Mailboxes for communication
// - Events for synchronization
// - Clock generation
// - DUT instance
//
module tb;

  //--------------------------------------------------
  // Verification Components
  //--------------------------------------------------

  generator gen;
  driver drv;
  monitor mon;
  scoreboard sco;

  //--------------------------------------------------
  // Synchronization Events
  //--------------------------------------------------

  // Driver -> Generator synchronization
  event nextgd;

  // Scoreboard -> Generator synchronization
  event nextgs;

  //--------------------------------------------------
  // Mailboxes
  //--------------------------------------------------

  // Generator -> Driver mailbox
  mailbox #(transaction) mbxgd;

  // Monitor -> Scoreboard mailbox
  mailbox #(transaction) mbxms;

  //--------------------------------------------------
  // Interface Instance
  //--------------------------------------------------

  i2c_if vif();

  //--------------------------------------------------
  // DUT Instance
  //--------------------------------------------------
  //
  // Connect DUT ports to interface signals
  //
  i2c_top dut (
    vif.clk,
    vif.rst,
    vif.newd,
    vif.op,
    vif.addr,
    vif.din,
    vif.dout,
    vif.busy,
    vif.ack_err,
    vif.done
  );

  //--------------------------------------------------
  // Clock Initialization
  //--------------------------------------------------

  initial begin
    vif.clk <= 0;
  end

  //--------------------------------------------------
  // Clock Generation
  //--------------------------------------------------
  //
  // Clock Period = 10 ns
  // Frequency = 100 MHz
  //
  always #5 vif.clk <= ~vif.clk;

  //--------------------------------------------------
  // Build Phase
  //--------------------------------------------------
  //
  // Create all testbench objects and connect them.
  //
  initial begin

    //----------------------------------------------
    // Create Mailboxes
    //----------------------------------------------
    mbxgd = new();
    mbxms = new();

    //----------------------------------------------
    // Create Components
    //----------------------------------------------
    gen = new(mbxgd);
    drv = new(mbxgd);

    mon = new(mbxms);
    sco = new(mbxms);

    //----------------------------------------------
    // Number of transactions to generate
    //----------------------------------------------
    gen.count = 20;

    //----------------------------------------------
    // Connect Virtual Interfaces
    //----------------------------------------------
    drv.vif = vif;
    mon.vif = vif;

    //----------------------------------------------
    // Connect Driver Synchronization Event
    //----------------------------------------------
    gen.drvnext = nextgd;
    drv.drvnext = nextgd;

    //----------------------------------------------
    // Connect Scoreboard Synchronization Event
    //----------------------------------------------
    gen.sconext = nextgs;
    sco.sconext = nextgs;

  end

  //--------------------------------------------------
  // Pre-Test Phase
  //--------------------------------------------------
  //
  // Perform DUT reset before test starts.
  //
  task pre_test;
    drv.reset();
  endtask

  //--------------------------------------------------
  // Test Phase
  //--------------------------------------------------
  //
  // Run all verification components simultaneously.
  //
  task test;

    fork

      gen.run();
      drv.run();
      mon.run();
      sco.run();

    join_any

  endtask

  //--------------------------------------------------
  // Post-Test Phase
  //--------------------------------------------------
  //
  // Wait until generator completes all transactions.
  //
  task post_test;

    wait(gen.done.triggered);

    $finish();

  endtask

  //--------------------------------------------------
  // Main Test Sequence
  //--------------------------------------------------
  task run();

    pre_test();

    test();

    post_test();

  endtask

  //--------------------------------------------------
  // Start Test
  //--------------------------------------------------
  initial begin
    run();
  end

  //--------------------------------------------------
  // Waveform Dump
  //--------------------------------------------------
  //
  // Generates VCD file for waveform viewing.
  //
  initial begin

    $dumpfile("dump.vcd");

    $dumpvars();

  end

endmodule
